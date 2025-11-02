SUMMARY = "Firmware for Realtek RTL8188EU"
LICENSE = "Proprietary"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/Proprietary;md5=0557f9d92cf58f2ccdd50f62f8ac0b28"


SRC_URI = " \
    file://rtl8188eufw.bin \
    file://rtl8192eu_nic.bin \
"

S = "${WORKDIR}"

do_install() {
    install -d ${D}/lib/firmware/rtlwifi
    install -m 0644 ${WORKDIR}/rtl8188eufw.bin ${D}/lib/firmware/rtlwifi/
    install -m 0644 ${WORKDIR}/rtl8192eu_nic.bin ${D}/lib/firmware/rtlwifi/
}

FILES:${PN} += " \
    /lib/firmware/rtlwifi/rtl8188eufw.bin \
    /lib/firmware/rtlwifi/rtl8192eu_nic.bin \
"

