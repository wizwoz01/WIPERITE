\
/*
 * OV7670 Camera Tracking with OpenCV (HSV threshold) for Zynq-7000 PetaLinux
 * Userspace, DMA S2MM + /dev/mem + I2C + OpenCV (core,imgproc,imgcodecs)
 * MIT License
 */
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <utility>
#include <stdexcept>
#include <chrono>

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#ifndef I2C_SLAVE
#define I2C_SLAVE 0x0703
#endif

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

// ------------------ AXI DMA registers (S2MM) ------------------
static constexpr uint32_t S2MM_DMACR   = 0x30; // Control
static constexpr uint32_t S2MM_DMASR   = 0x34; // Status
static constexpr uint32_t S2MM_DA      = 0x48; // Dest Addr Low
static constexpr uint32_t S2MM_DA_MSB  = 0x4C; // Dest Addr High
static constexpr uint32_t S2MM_LENGTH  = 0x58; // Transfer length (bytes)

static inline void mmio_write32(volatile uint8_t* base, uint32_t off, uint32_t v){
    *(volatile uint32_t*)(base + off) = v;
    asm volatile("" ::: "memory");
}
static inline uint32_t mmio_read32(volatile uint8_t* base, uint32_t off){
    asm volatile("" ::: "memory");
    return *(volatile uint32_t*)(base + off);
}

// ------------------ CLI opts ------------------
struct Opts {
    int width  = 640;
    int height = 480;
    uint32_t fb_phys  = 0x1F000000;
    uint32_t dma_phys = 0x40400000;
    int i2c_bus  = 0;
    int i2c_addr = 0x21; // 7-bit
    int loops    = 0;    // 0 = infinite
    const char* out_path = nullptr; // save annotated BGR as PNG/PPM
    bool rgb565 = true;  // if false, interpret as BGR565
    // HSV thresholds (tuned for "orange" object)
    int h_lo = 5,  s_lo = 80,  v_lo = 80;
    int h_hi = 25, s_hi = 255, v_hi = 255;
};

static void usage(const char* prog){
    std::fprintf(stderr,
        "Usage: %s [options]\n\n"
        "  --width <W>           Frame width (default 640)\n"
        "  --height <H>          Frame height (default 480)\n"
        "  --fb-phys <addr>      Phys address of capture buffer (default 0x1F000000)\n"
        "  --dma-phys <addr>     Phys address of AXI DMA S2MM regs (default 0x40400000)\n"
        "  --i2c-bus <n>         I2C bus index (default 0)\n"
        "  --i2c-addr <0xNN>     I2C 7-bit address (default 0x21)\n"
        "  --loops <N>           Frames to capture (default 0 = infinite)\n"
        "  --out <path>          Save last annotated frame (PNG/PPM)\n"
        "  --rgb565/--bgr565     Select input packing (default --rgb565)\n"
        "  --hlo <0-179> --hhi <0-179>\n"
        "  --slo <0-255> --shi <0-255>\n"
        "  --vlo <0-255> --vhi <0-255>\n"
        "  --help\n", prog);
}

static bool parse_hex(const char* s, uint32_t& out){
    char* end=nullptr;
    unsigned long v = std::strtoul(s, &end, 0);
    if(end==s) return false;
    out = (uint32_t)v;
    return true;
}

static Opts parse_opts(int argc, char** argv){
    Opts o;
    for (int i=1;i<argc;i++){
        std::string a = argv[i];
        auto need = [&](int i){
            if (i+1>=argc) { std::fprintf(stderr, "Missing value after %s\n", a.c_str()); std::exit(1); }
        };
        if (a=="--help"){ usage(argv[0]); std::exit(0); }
        else if (a=="--width"){ need(i); o.width = std::atoi(argv[++i]); }
        else if (a=="--height"){ need(i); o.height = std::atoi(argv[++i]); }
        else if (a=="--fb-phys"){ need(i); if(!parse_hex(argv[++i], o.fb_phys)){ std::fprintf(stderr,"bad addr\n"); std::exit(1);} }
        else if (a=="--dma-phys"){ need(i); if(!parse_hex(argv[++i], o.dma_phys)){ std::fprintf(stderr,"bad addr\n"); std::exit(1);} }
        else if (a=="--i2c-bus"){ need(i); o.i2c_bus = std::atoi(argv[++i]); }
        else if (a=="--i2c-addr"){ need(i); uint32_t v; if(!parse_hex(argv[++i], v)){ std::fprintf(stderr,"bad addr\n"); std::exit(1);} o.i2c_addr=(int)v; }
        else if (a=="--loops"){ need(i); o.loops = std::atoi(argv[++i]); }
        else if (a=="--out"){ need(i); o.out_path = argv[++i]; }
        else if (a=="--rgb565"){ o.rgb565 = true; }
        else if (a=="--bgr565"){ o.rgb565 = false; }
        else if (a=="--hlo"){ need(i); o.h_lo = std::atoi(argv[++i]); }
        else if (a=="--hhi"){ need(i); o.h_hi = std::atoi(argv[++i]); }
        else if (a=="--slo"){ need(i); o.s_lo = std::atoi(argv[++i]); }
        else if (a=="--shi"){ need(i); o.s_hi = std::atoi(argv[++i]); }
        else if (a=="--vlo"){ need(i); o.v_lo = std::atoi(argv[++i]); }
        else if (a=="--vhi"){ need(i); o.v_hi = std::atoi(argv[++i]); }
        else { std::fprintf(stderr, "Unknown option: %s\n", a.c_str()); usage(argv[0]); std::exit(1); }
    }
    return o;
}

// ------------------ Helpers ------------------
static int open_i2c(int bus, int addr7){
    char path[32]; std::snprintf(path, sizeof(path), "/dev/i2c-%d", bus);
    int fd = ::open(path, O_RDWR);
    if (fd<0) { std::perror("open i2c"); throw std::runtime_error("open i2c"); }
    if (ioctl(fd, I2C_SLAVE, addr7)<0){ std::perror("ioctl I2C_SLAVE"); throw std::runtime_error("i2c slave"); }
    return fd;
}
static void i2c_write_reg(int fd, uint8_t reg, uint8_t val){
    uint8_t buf[2] = {reg, val};
    if (write(fd, buf, 2)!=2) { std::perror("i2c write"); throw std::runtime_error("i2c write"); }
    usleep(2000);
}
static int open_devmem(){ int fd = ::open("/dev/mem", O_RDWR|O_SYNC); if(fd<0){ std::perror("open /dev/mem"); throw std::runtime_error("devmem"); } return fd; }
static volatile uint8_t* map_phys(int fd, uint32_t phys, size_t len){
    void* va = ::mmap(nullptr, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, phys);
    if (va==MAP_FAILED){ std::perror("mmap"); throw std::runtime_error("mmap"); }
    return (volatile uint8_t*)va;
}

// ------------------ OV7670 init table ------------------
static std::vector<std::pair<uint8_t,uint8_t>> ov7670_regs_rgb565_vga = {
    {0x12, 0x80}, // reset
    {0x12, 0x14}, // COM7: RGB, VGA
    {0x8C, 0x00}, // RGB444 off
    {0x40, 0x10}, // COM15: RGB565
    {0x3A, 0x04}, // TSLB byte order for RGB
    {0x11, 0x01}, {0x6B, 0x4A}, {0x3D, 0xC0},
    {0x0C, 0x00}, {0x3E, 0x00},
    {0x70, 0x00}, {0x71, 0x00}, {0x72, 0x11}, {0x73, 0x00},
    {0xA2, 0x02},
};
static std::vector<std::pair<uint8_t,uint8_t>> ov7670_regs_rgb565_qvga = {
    {0x12, 0x14},
    {0x8C, 0x00},
    {0x40, 0x10},
    {0x3A, 0x04},
    {0x11, 0x01}, {0x6B, 0x4A}, {0x3D, 0xC0},
    {0x0C, 0x04}, // scaling
    {0x3E, 0x19},
    {0x70, 0x3A}, {0x71, 0x35}, {0x72, 0x11}, {0x73, 0xF1},
    {0xA2, 0x02},
};
static void ov7670_init(int i2cfd, int W, int H){
    const auto &tbl = (W==320 && H==240) ? ov7670_regs_rgb565_qvga : ov7670_regs_rgb565_vga;
    for (auto &kv : tbl) { i2c_write_reg(i2cfd, kv.first, kv.second); }
}

// ------------------ DMA control ------------------
static void dma_reset_and_run(volatile uint8_t* dma){
    mmio_write32(dma, S2MM_DMACR, (1u<<2)); // reset
    usleep(1000);
    mmio_write32(dma, S2MM_DMACR, 0x0001 | (1u<<12) | (1u<<14)); // run + IRQ enable
    mmio_write32(dma, S2MM_DMASR, 0xFFFFFFFF); // ack all
}
static void dma_start_frame(volatile uint8_t* dma, uint32_t dst, uint32_t length){
    mmio_write32(dma, S2MM_DA,     dst);
    mmio_write32(dma, S2MM_DA_MSB, 0);
    mmio_write32(dma, S2MM_LENGTH, length);
}
static void dma_wait_done(volatile uint8_t* dma){
    for(;;){
        uint32_t sr = mmio_read32(dma, S2MM_DMASR);
        if (sr & (1u<<12)){ mmio_write32(dma, S2MM_DMASR, (1u<<12)); break; }
        if (sr & ((1u<<14)|(1u<<28))){ std::fprintf(stderr, "DMA error SR=0x%08x\n", sr); throw std::runtime_error("DMA error"); }
        usleep(1000);
    }
}

// ------------------ main ------------------
int main(int argc, char** argv){
    Opts o = parse_opts(argc, argv);
    const uint32_t frame_bytes = (uint32_t)(o.width * o.height * 2);

    int memfd = open_devmem();
    volatile uint8_t* dma = map_phys(memfd, o.dma_phys, 0x10000);
    volatile uint8_t* fb  = map_phys(memfd, o.fb_phys,  frame_bytes);

    int i2cfd = open_i2c(o.i2c_bus, o.i2c_addr);
    i2c_write_reg(i2cfd, 0x12, 0x80); // reset
    usleep(50000);
    ov7670_init(i2cfd, o.width, o.height);
    close(i2cfd);

    dma_reset_and_run(dma);

    int frame = 0;
    while (true){
        if (o.loops>0 && frame>=o.loops) break;

        auto t1 = std::chrono::high_resolution_clock::now();
        dma_start_frame(dma, o.fb_phys, frame_bytes);
        dma_wait_done(dma);
        auto t2 = std::chrono::high_resolution_clock::now();

        // Wrap RGB565 buffer as CV_8UC2
        cv::Mat m565(o.height, o.width, CV_8UC2, (void*)fb);
        cv::Mat bgr;
        if (o.rgb565){
            // OpenCV provides both RGB565 and BGR565 conversions; pick via flag.
            cv::cvtColor(m565, bgr, cv::COLOR_RGB5652BGR);
        } else {
            cv::cvtColor(m565, bgr, cv::COLOR_BGR5652BGR);
        }

        // HSV threshold
        cv::Mat hsv, mask;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv,
                    cv::Scalar(o.h_lo, o.s_lo, o.v_lo),
                    cv::Scalar(o.h_hi, o.s_hi, o.v_hi),
                    mask);
        // Morphological clean-up (optional)
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

        // Centroid via moments
        cv::Moments mu = cv::moments(mask, true);
        int cx=-1, cy=-1;
        if (mu.m00 > 0.0){
            cx = int(mu.m10 / mu.m00);
            cy = int(mu.m01 / mu.m00);
            cv::circle(bgr, cv::Point(cx,cy), 8, cv::Scalar(0,255,0), 2);
            cv::putText(bgr, std::to_string(cx)+","+std::to_string(cy),
                        cv::Point(cx+10, cy-10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
        }

        auto t3 = std::chrono::high_resolution_clock::now();
        double cap_ms  = std::chrono::duration<double, std::milli>(t2 - t1).count();
        double proc_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

        std::printf("frame=%d centroid=(%d,%d) cap_ms=%.1f proc_ms=%.1f\n",
                    frame, cx, cy, cap_ms, proc_ms);
        std::fflush(stdout);

        frame++;
    }

    if (o.out_path){
        // Save last annotated BGR frame
        // If cv::imwrite can't determine format, use .png or .ppm
        cv::Mat m565(o.height, o.width, CV_8UC2, (void*)fb);
        cv::Mat bgr;
        if (o.rgb565) cv::cvtColor(m565, bgr, cv::COLOR_RGB5652BGR);
        else          cv::cvtColor(m565, bgr, cv::COLOR_BGR5652BGR);
        cv::imwrite(o.out_path, bgr);
    }
    return 0;
}
