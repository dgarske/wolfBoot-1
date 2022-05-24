# Intel FSP Binaries

These binaries were collected and build using the Intel slimbootloader project.

Intel Slim Bootlaoder https://github.com/slimbootloader/slimbootloader
Intel FSP: https://github.com/intel/FSP


## Building the FSP Binaries

Instructions from: https://slimbootloader.github.io/supported-hardware/qemu.html#

```sh
git clone https://github.com/slimbootloader/slimbootloader.git
cd slimbootloader
export SBL_ROOT=`pwd`
export SBL_KEY_DIR=$SBL_ROOT/SblKeys/
python3 ./BootloaderCorePkg/Tools/GenerateKeys.py -k $SBL_KEY_DIR
python3 BuildLoader.py build qemu
OR
python3 BuildLoader.py build tgl
```


Generated binaries are here:

```sh
$ find . -name FspRel.bin
./Silicon/TigerlakePkg/FspBin/FspRel.bin
./Silicon/QemuSocPkg/FspBin/FspRel.bin
```

## QEMU Testing

```sh
qemu-system-x86_64 -machine q35 -nographic -serial mon:stdio -pflash Outputs/qemu/SlimBootloader.bin
```

Note: To exit use Shift+Ctrl+A then x
