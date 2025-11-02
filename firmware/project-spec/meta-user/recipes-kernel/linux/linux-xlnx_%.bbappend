FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append = " file://bsp.cfg"
KERNEL_FEATURES:append = " bsp.cfg"
SRC_URI += "file://user_2025-06-14-19-27-00.cfg \
            file://user_2025-06-26-22-38-00.cfg \
            file://user_2025-06-27-03-14-00.cfg \
            file://user_2025-10-12-01-44-00.cfg \
            file://user_2025-10-23-06-32-00.cfg \
            file://user_2025-10-23-07-43-00.cfg \
            file://user_2025-10-23-22-48-00.cfg \
            file://user_2025-10-26-00-31-00.cfg \
            file://user_2025-10-26-13-49-00.cfg \
            "

