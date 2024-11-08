; ModuleID = '/home/samuel/VitisHLS/assig2/image_filter/solution1/.autopilot/db/a.g.ld.5.gdce.bc'
source_filename = "llvm-link"
target datalayout = "e-m:e-i64:64-i128:128-i256:256-i512:512-i1024:1024-i2048:2048-i4096:4096-n8:16:32:64-S128-v16:16-v24:32-v32:32-v48:64-v96:128-v192:256-v256:256-v512:512-v1024:1024"
target triple = "fpga64-xilinx-none"

; Function Attrs: noinline
define void @apatb_filterImage_ir(i8* %rowBelow, i8* %rowCenter, i8* %rowAbove, i8* %outputRow) local_unnamed_addr #0 {
entry:
  %rowBelow_copy = alloca [120 x i8], align 512
  %rowCenter_copy = alloca [120 x i8], align 512
  %rowAbove_copy = alloca [120 x i8], align 512
  %outputRow_copy = alloca [120 x i8], align 512
  %0 = bitcast i8* %rowBelow to [120 x i8]*
  %1 = bitcast i8* %rowCenter to [120 x i8]*
  %2 = bitcast i8* %rowAbove to [120 x i8]*
  %3 = bitcast i8* %outputRow to [120 x i8]*
  call fastcc void @copy_in([120 x i8]* %0, [120 x i8]* nonnull align 512 %rowBelow_copy, [120 x i8]* %1, [120 x i8]* nonnull align 512 %rowCenter_copy, [120 x i8]* %2, [120 x i8]* nonnull align 512 %rowAbove_copy, [120 x i8]* %3, [120 x i8]* nonnull align 512 %outputRow_copy)
  %4 = getelementptr inbounds [120 x i8], [120 x i8]* %rowBelow_copy, i32 0, i32 0
  %5 = getelementptr inbounds [120 x i8], [120 x i8]* %rowCenter_copy, i32 0, i32 0
  %6 = getelementptr inbounds [120 x i8], [120 x i8]* %rowAbove_copy, i32 0, i32 0
  %7 = getelementptr inbounds [120 x i8], [120 x i8]* %outputRow_copy, i32 0, i32 0
  call void @apatb_filterImage_hw(i8* %4, i8* %5, i8* %6, i8* %7)
  call fastcc void @copy_out([120 x i8]* %0, [120 x i8]* nonnull align 512 %rowBelow_copy, [120 x i8]* %1, [120 x i8]* nonnull align 512 %rowCenter_copy, [120 x i8]* %2, [120 x i8]* nonnull align 512 %rowAbove_copy, [120 x i8]* %3, [120 x i8]* nonnull align 512 %outputRow_copy)
  ret void
}

; Function Attrs: argmemonly noinline
define internal fastcc void @copy_in([120 x i8]* readonly, [120 x i8]* noalias align 512, [120 x i8]* readonly, [120 x i8]* noalias align 512, [120 x i8]* readonly, [120 x i8]* noalias align 512, [120 x i8]* readonly, [120 x i8]* noalias align 512) unnamed_addr #1 {
entry:
  call fastcc void @onebyonecpy_hls.p0a120i8([120 x i8]* align 512 %1, [120 x i8]* %0)
  call fastcc void @onebyonecpy_hls.p0a120i8([120 x i8]* align 512 %3, [120 x i8]* %2)
  call fastcc void @onebyonecpy_hls.p0a120i8([120 x i8]* align 512 %5, [120 x i8]* %4)
  call fastcc void @onebyonecpy_hls.p0a120i8([120 x i8]* align 512 %7, [120 x i8]* %6)
  ret void
}

; Function Attrs: argmemonly noinline
define internal fastcc void @onebyonecpy_hls.p0a120i8([120 x i8]* noalias align 512, [120 x i8]* noalias readonly) unnamed_addr #2 {
entry:
  %2 = icmp eq [120 x i8]* %0, null
  %3 = icmp eq [120 x i8]* %1, null
  %4 = or i1 %2, %3
  br i1 %4, label %ret, label %copy

copy:                                             ; preds = %entry
  br label %for.loop

for.loop:                                         ; preds = %for.loop, %copy
  %for.loop.idx1 = phi i64 [ 0, %copy ], [ %for.loop.idx.next, %for.loop ]
  %dst.addr = getelementptr [120 x i8], [120 x i8]* %0, i64 0, i64 %for.loop.idx1
  %src.addr = getelementptr [120 x i8], [120 x i8]* %1, i64 0, i64 %for.loop.idx1
  call void @llvm.memcpy.p0i8.p0i8.i64(i8* align 1 %dst.addr, i8* align 1 %src.addr, i64 1, i1 false)
  %for.loop.idx.next = add nuw nsw i64 %for.loop.idx1, 1
  %exitcond = icmp ne i64 %for.loop.idx.next, 120
  br i1 %exitcond, label %for.loop, label %ret

ret:                                              ; preds = %for.loop, %entry
  ret void
}

; Function Attrs: argmemonly nounwind
declare void @llvm.memcpy.p0i8.p0i8.i64(i8* nocapture writeonly, i8* nocapture readonly, i64, i1) #3

; Function Attrs: argmemonly noinline
define internal fastcc void @copy_out([120 x i8]*, [120 x i8]* noalias readonly align 512, [120 x i8]*, [120 x i8]* noalias readonly align 512, [120 x i8]*, [120 x i8]* noalias readonly align 512, [120 x i8]*, [120 x i8]* noalias readonly align 512) unnamed_addr #4 {
entry:
  call fastcc void @onebyonecpy_hls.p0a120i8([120 x i8]* %0, [120 x i8]* align 512 %1)
  call fastcc void @onebyonecpy_hls.p0a120i8([120 x i8]* %2, [120 x i8]* align 512 %3)
  call fastcc void @onebyonecpy_hls.p0a120i8([120 x i8]* %4, [120 x i8]* align 512 %5)
  call fastcc void @onebyonecpy_hls.p0a120i8([120 x i8]* %6, [120 x i8]* align 512 %7)
  ret void
}

declare void @apatb_filterImage_hw(i8*, i8*, i8*, i8*)

define void @filterImage_hw_stub_wrapper(i8*, i8*, i8*, i8*) #5 {
entry:
  %4 = bitcast i8* %0 to [120 x i8]*
  %5 = bitcast i8* %1 to [120 x i8]*
  %6 = bitcast i8* %2 to [120 x i8]*
  %7 = bitcast i8* %3 to [120 x i8]*
  call void @copy_out([120 x i8]* null, [120 x i8]* %4, [120 x i8]* null, [120 x i8]* %5, [120 x i8]* null, [120 x i8]* %6, [120 x i8]* null, [120 x i8]* %7)
  %8 = bitcast [120 x i8]* %4 to i8*
  %9 = bitcast [120 x i8]* %5 to i8*
  %10 = bitcast [120 x i8]* %6 to i8*
  %11 = bitcast [120 x i8]* %7 to i8*
  call void @filterImage_hw_stub(i8* %8, i8* %9, i8* %10, i8* %11)
  call void @copy_in([120 x i8]* null, [120 x i8]* %4, [120 x i8]* null, [120 x i8]* %5, [120 x i8]* null, [120 x i8]* %6, [120 x i8]* null, [120 x i8]* %7)
  ret void
}

declare void @filterImage_hw_stub(i8*, i8*, i8*, i8*)

attributes #0 = { noinline "fpga.wrapper.func"="wrapper" }
attributes #1 = { argmemonly noinline "fpga.wrapper.func"="copyin" }
attributes #2 = { argmemonly noinline "fpga.wrapper.func"="onebyonecpy_hls" }
attributes #3 = { argmemonly nounwind }
attributes #4 = { argmemonly noinline "fpga.wrapper.func"="copyout" }
attributes #5 = { "fpga.wrapper.func"="stub" }

!llvm.dbg.cu = !{}
!llvm.ident = !{!0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0}
!llvm.module.flags = !{!1, !2, !3}
!blackbox_cfg = !{!4}

!0 = !{!"clang version 7.0.0 "}
!1 = !{i32 2, !"Dwarf Version", i32 4}
!2 = !{i32 2, !"Debug Info Version", i32 3}
!3 = !{i32 1, !"wchar_size", i32 4}
!4 = !{}
