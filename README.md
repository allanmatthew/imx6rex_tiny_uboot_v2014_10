# imx6rex_tiny_uboot_v2014_10
mx6q/dl/s rex tiny u-boot v2014.10 

# Download repository
    git clone https://github.com/voipac/imx6rex_tiny_uboot_v2014_10
    cd imx6rex_tiny_uboot_v2014_10

# Setup cross compiler
    export PATH="/opt/gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux/bin:~/workdir/bin:$PATH"
    export CROSS_COMPILE=arm-linux-gnueabihf-
    export ARCH=arm

# Build (imx6s)
    make distclean
    make mx6srextiny_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-rex-tiny.imx
    
# Build (imx6dl)
    TODO

# Build (imx6q)
    TODO
