start:
	push %rbp
	mov  %rsp,%rbp
	mov    $0x2000,%esi
	mov    $0x604,%edi
	sub    $0x8,%rsp
	mov    %edi,%edx
   	mov    %esi,%eax
   	mov    %dx,-0x4(%rbp)
   	mov    %ax,-0x8(%rbp)
   	movzwl -0x8(%rbp),%eax
   	movzwl -0x4(%rbp),%edx
   	out    %ax,(%dx)
	mov    $0x29,%esi
	mov    $0x501,%edi
	mov    %edi,%edx
   	mov    %esi,%eax
   	mov    %dx,-0x4(%rbp)
   	mov    %ax,-0x8(%rbp)
   	movzwl -0x8(%rbp),%eax
   	movzwl -0x4(%rbp),%edx
   	out    %ax,(%dx)
	leaveq
	retq
