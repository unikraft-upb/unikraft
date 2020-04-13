all: obj link1 link2 link3 link4

obj:
	gcc -I./include -DCC_VERSION=7.5 -D__X86_64__ -m64 -U __linux__ -U __FreeBSD__ \
	-U __sun__ -D__ASSEMBLY__ -D __Unikraft__ -DUK_CODENAME="Rhea" -DUK_VERSION=0.4 -DUK_FULLVERSION=0.4.0~00bbf2c \
	-mtune=generic -no-pie -g3 -DKVMPLAT -D__LIBNAME__=libkvmplat -D__BASENAME__=entry64.S -c entry64.S \
	-o entry64.o -Wp,-MD,.entry64.o.d

link1:
	gcc -Wl,--defsym=main=_libkvmplat_start32 -no-pie entry64.o -o entry1

link2:
	gcc -nostdinc -nostdlib -Wl,--defsym=main=_libkvmplat_start32 -no-pie entry64.o -o entry2

link3:
	ld -s -static entry64.o -e _libkvmplat_start32 -lc -o entry3

link4:
	ld --defsym=main=_libkvmplat_start32 -no-pie entry64.o -o entry4

run:
	sudo qemu-system-x86_64 -enable-kvm -nographic -device isa-debug-exit -kernel $(exec)

run_gdb:
	sudo qemu-system-x86_64 -s -S -cpu host -enable-kvm -m 128 -nodefaults -no-acpi -display none -serial stdio -device isa-debug-exit -kernel $(exec) -append verbose

clean:
	rm -rf .entry64.o.d entry64.o entry1 entry2 entry3 entry4
