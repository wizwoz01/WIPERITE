\
/*
 * OV7670 Camera Tracking (RGB565) for Zynq-7000 PetaLinux
 * Userspace, no OpenCV, DMA S2MM + /dev/mem + I2C
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

// Minimal i2c-dev (avoid header dependency)
#ifndef I2C_SLAVE
#define I2C_SLAVE 0x0703
#endif

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
    const char* ppm_path = nullptr;
    // Color threshold (RGB565 downsampled ranges)
    int rmin = 21, gmin = 10, gmax = 40, bmax = 10;
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
        "  --ppm <path>          Write last frame to PPM (optional)\n"
        "  --rmin <0-31>         Red min (565 range) default 21\n"
        "  --gmin <0-63>         Green min default 10\n"
        "  --gmax <0-63>         Green max default 40\n"
        "  --bmax <0-31>         Blue max default 10\n"
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
        else if (a=="--ppm"){ need(i); o.ppm_path = argv[++i]; }
        else if (a=="--rmin"){ need(i); o.rmin = std::atoi(argv[++i]); }
        else if (a=="--gmin"){ need(i); o.gmin = std::atoi(argv[++i]); }
        else if (a=="--gmax"){ need(i); o.gmax = std::atoi(argv[++i]); }
        else if (a=="--bmax"){ need(i); o.bmax = std::atoi(argv[++i]); }
        else {
            std::fprintf(stderr, "Unknown option: %s\n", a.c_str());
            usage(argv[0]); std::exit(1);
        }
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
    {0x12, 0x80}, // COM7 reset
    // RGB565
    {0x12, 0x14}, // COM7: RGB, VGA
    {0x8C, 0x00}, // RGB444 off
    {0x40, 0x10}, // COM15: RGB565, full range
    {0x3A, 0x04}, // TSLB: UYVY order fix for RGB
    {0x11, 0x01}, // CLKRC: internal pre-scaler
    {0x6B, 0x4A}, // PLL (trial)
    {0x3D, 0xC0}, // COM13
    {0x0C, 0x00}, // COM3: scaling off
    {0x3E, 0x00}, // COM14: manual scaling off
    {0x70, 0x00}, {0x71, 0x00}, {0x72, 0x11}, {0x73, 0x00},
    {0xA2, 0x02},
};
static std::vector<std::pair<uint8_t,uint8_t>> ov7670_regs_rgb565_qvga = {
    {0x12, 0x14}, // COM7: RGB
    {0x8C, 0x00},
    {0x40, 0x10}, // RGB565
    {0x3A, 0x04},
    {0x11, 0x01},
    {0x6B, 0x4A},
    {0x3D, 0xC0},
    {0x0C, 0x04}, // scaling on
    {0x3E, 0x19}, // manual scaling + pclk div
    {0x70, 0x3A}, {0x71, 0x35}, {0x72, 0x11}, {0x73, 0xF1},
    {0xA2, 0x02},
};

static void ov7670_init(int i2cfd, int W, int H){
    const auto &tbl = (W==320 && H==240) ? ov7670_regs_rgb565_qvga : ov7670_regs_rgb565_vga;
    for (auto &kv : tbl) { i2c_write_reg(i2cfd, kv.first, kv.second); }
}

// ------------------ DMA control ------------------
static void dma_reset_and_run(volatile uint8_t* dma){
    // Reset
    mmio_write32(dma, S2MM_DMACR, (1u<<2)); // Reset bit
    usleep(1000);
    // Run + enable IRQs (even if not used here)
    mmio_write32(dma, S2MM_DMACR, 0x0001 | (1u<<12) | (1u<<14));
    // Clear pending status
    mmio_write32(dma, S2MM_DMASR, 0xFFFFFFFF);
}
static void dma_start_frame(volatile uint8_t* dma, uint32_t dst, uint32_t length){
    mmio_write32(dma, S2MM_DA,     dst);
    mmio_write32(dma, S2MM_DA_MSB, 0);
    mmio_write32(dma, S2MM_LENGTH, length);
}
static void dma_wait_done(volatile uint8_t* dma){
    // Wait for IOC_Irq (bit 12)
    for(;;){
        uint32_t sr = mmio_read32(dma, S2MM_DMASR);
        if (sr & (1u<<12)){
            // ack
            mmio_write32(dma, S2MM_DMASR, (1u<<12));
            break;
        }
        if (sr & ((1u<<14)|(1u<<28))){
            std::fprintf(stderr, "DMA error, DMASR=0x%08x\n", sr);
            throw std::runtime_error("DMA error");
        }
        // light poll
        usleep(1000);
    }
}

// ------------------ Tracking ------------------
struct Centroid { int x=-1, y=-1, n=0; };

static Centroid track_rgb565(const uint16_t* img, int W, int H, int rmin, int gmin, int gmax, int bmax){
    long long sx=0, sy=0; int n=0;
    for (int y=0; y<H; ++y){
        const uint16_t* row = img + y*W;
        for (int x=0; x<W; ++x){
            uint16_t p = row[x];
            int r = (p>>11) & 0x1F;
            int g = (p>>5)  & 0x3F;
            int b = (p)     & 0x1F;
            if (r>=rmin && g>=gmin && g<=gmax && b<=bmax){ sx += x; sy += y; ++n; }
        }
    }
    Centroid c;
    c.n = n;
    if (n>0){ c.x = (int)(sx / n); c.y = (int)(sy / n); }
    return c;
}

static void write_ppm_rgb565(const char* path, const uint16_t* img, int W, int H){
    FILE* f = std::fopen(path, "wb");
    if (!f){ std::perror("fopen ppm"); return; }
    std::fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (int i=0;i<W*H;i++){
        uint16_t p = img[i];
        uint8_t r = ((p>>11)&0x1F)<<3;
        uint8_t g = ((p>>5)&0x3F)<<2;
        uint8_t b = (p&0x1F)<<3;
        std::fputc(r, f); std::fputc(g, f); std::fputc(b, f);
    }
    std::fclose(f);
}

// ------------------ main ------------------
int main(int argc, char** argv){
    Opts o = parse_opts(argc, argv);
    const uint32_t frame_bytes = (uint32_t)(o.width * o.height * 2);
    // Map regions
    int memfd = open_devmem();
    volatile uint8_t* dma = map_phys(memfd, o.dma_phys, 0x10000);
    volatile uint8_t* fb  = map_phys(memfd, o.fb_phys,  frame_bytes);

    // I2C configure OV7670
    int i2cfd = open_i2c(o.i2c_bus, o.i2c_addr);
    try{
        // Reset then init
        i2c_write_reg(i2cfd, 0x12, 0x80);
        usleep(50000);
        ov7670_init(i2cfd, o.width, o.height);
    } catch(...){
        std::fprintf(stderr, "OV7670 init failed.\n");
    }
    close(i2cfd);

    // Loop frames
    dma_reset_and_run(dma);
    int frame = 0;
    auto t0 = std::chrono::high_resolution_clock::now();
    while (true){
        if (o.loops>0 && frame>=o.loops) break;
        auto t1 = std::chrono::high_resolution_clock::now();
        dma_start_frame(dma, o.fb_phys, frame_bytes);
        dma_wait_done(dma);
        auto t2 = std::chrono::high_resolution_clock::now();

        // Process
        const uint16_t* img = (const uint16_t*)fb;
        Centroid c = track_rgb565(img, o.width, o.height, o.rmin, o.gmin, o.gmax, o.bmax);
        auto t3 = std::chrono::high_resolution_clock::now();

        double cap_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        double proc_ms= std::chrono::duration<double, std::milli>(t3 - t2).count();

        std::printf("frame=%d mask=%d centroid=(%d,%d) cap_ms=%.1f proc_ms=%.1f\n",
            frame, c.n, c.x, c.y, cap_ms, proc_ms);
        std::fflush(stdout);

        frame++;
    }

    if (o.ppm_path){
        write_ppm_rgb565(o.ppm_path, (const uint16_t*)fb, o.width, o.height);
    }
    return 0;
}
