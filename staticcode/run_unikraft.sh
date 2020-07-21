sudo qemu-system-x86_64 -m 2G -enable-kvm -nographic -nodefaults -display none -serial stdio -device \
isa-debug-exit -kernel ~/Faculty/BachelorThesis/upbbranch/unikraft/staticcode/bootloader.dbg -initrd \
$1
