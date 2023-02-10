/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package xiangshan.backend.fu.fpu

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import utility.{SignExt, ZeroExt}
import xiangshan.backend.fu.HasPipelineReg
import xiangshan.{XSBundle, XSModule, i2fCfg}

class IntToVecBundle(implicit p: Parameters) extends XSBundle {
  // in
  val src0 = Input(UInt(64.W))
  val vWenIn = Input(Bool())
  val vstart = Input(UInt(64.W)) // If vstart ≥ vl, no operation is performed and the destination register is not updated.
  val vl = Input(UInt(8.W)) //
  val vdIn = Input(UInt(128.W))
  val vsew = Input(UInt(3.W)) // SEW = 2^(3+vsew[2:0]) 0->8 1->16 11->64  larger bit width for reservation
  val vta = Input(Bool())

  // out
  val vWenOut = Output(Bool())
  val vdOut = Output(UInt(128.W))
}

class IntToVecDataModule(latency: Int)(implicit p: Parameters) extends XSModule {

  val io = IO(new IntToVecBundle)

  io.vWenOut := Mux(io.vstart < io.vl, io.vWenIn, false.B)
  val vdOut0 = Mux1H(Seq(
    (io.vsew(1,0) === 0.U(2.W)) -> Cat(io.vdIn(127,8 ),io.src0(7 ,0)),
    (io.vsew(1,0) === 1.U(2.W)) -> Cat(io.vdIn(127,16),io.src0(15,0)),
    (io.vsew(1,0) === 2.U(2.W)) -> Cat(io.vdIn(127,32),io.src0(31,0)),
    (io.vsew(1,0) === 3.U(2.W)) -> Cat(io.vdIn(127,64),io.src0(63,0)),
  ))
  val vdOut1 = Mux1H(Seq(
    (io.vsew(1, 0) === 0.U(2.W)) -> Cat(Fill(120, 1.U(1.W)), io.src0(7, 0)),
    (io.vsew(1, 0) === 1.U(2.W)) -> Cat(Fill(112, 1.U(1.W)), io.src0(15, 0)),
    (io.vsew(1, 0) === 2.U(2.W)) -> Cat(Fill(96, 1.U(1.W)), io.src0(31, 0)),
    (io.vsew(1, 0) === 3.U(2.W)) -> Cat(Fill(64, 1.U(1.W)), io.src0(63, 0)),
  ))
  io.vdOut := Mux(io.vta, vdOut1, vdOut0)
}

class IntToFPDataModule(latency: Int)(implicit p: Parameters) extends FPUDataModule {
  val regEnables = IO(Input(Vec(latency, Bool())))
  //  val i2vIo = IO(new IntToVecBundle)
  //
  //  val i2v = Module(new IntToVecDataModule(0))
  //  i2v.io <> i2vIo
  //  stage1
  val ctrl = io.in.fpCtrl
  val in = io.in.src(0)
  val typ = ctrl.typ
  val intValue = RegEnable(Mux(ctrl.wflags,
    Mux(typ(1),
      Mux(typ(0), ZeroExt(in, XLEN), SignExt(in, XLEN)),
      Mux(typ(0), ZeroExt(in(31, 0), XLEN), SignExt(in(31, 0), XLEN))
    ),
    in
  ), regEnables(0))
  val ctrlReg = RegEnable(ctrl, regEnables(0))
  val rmReg = RegEnable(rm, regEnables(0))

  // stage2
  val s2_tag = ctrlReg.typeTagOut
  val s2_wflags = ctrlReg.wflags
  val s2_typ = ctrlReg.typ

  val mux = Wire(new Bundle() {
    val data = UInt(XLEN.W)
    val exc = UInt(5.W)
  })

  mux.data := intValue
  mux.exc := 0.U

  when(s2_wflags){
    val i2fResults = for(t <- FPU.ftypes) yield {
      val i2f = Module(new fudian.IntToFP(t.expWidth, t.precision))
      i2f.io.sign := ~s2_typ(0)
      i2f.io.long := s2_typ(1)
      i2f.io.int := intValue
      i2f.io.rm := rmReg
      (i2f.io.result, i2f.io.fflags)
    }
    val (data, exc) = i2fResults.unzip
    mux.data := VecInit(data)(s2_tag)
    mux.exc := VecInit(exc)(s2_tag)
  }

  // stage3
  val s3_out = RegEnable(mux, regEnables(1))
  val s3_tag = RegEnable(s2_tag, regEnables(1))

  fflags := s3_out.exc
  io.out.data := FPU.box(s3_out.data, s3_tag)
}

class IntToFP(implicit p: Parameters) extends FPUSubModule with HasPipelineReg {
  override def latency: Int = i2fCfg.latency.latencyVal.get
  override val dataModule = Module(new IntToFPDataModule(latency))
  //  TODO: add valid/ready signal
  //  dataModule.i2vIo.src0 := io.in.bits.src(0)
  //  dataModule.i2vIo.vWenIn := io.in.bits.uop.ctrl.vecWen
  //  dataModule.i2vIo.vstart := 0.U(64.W)
  //  dataModule.i2vIo.vl := io.in.bits.uop.ctrl.vconfig(15,8)
  //  //  dataModule.i2vIo.vdIn := _
  //  dataModule.i2vIo.vsew := io.in.bits.uop.ctrl.vconfig(5,3)
  //  dataModule.i2vIo.vta := io.in.bits.uop.ctrl.vconfig(6)
  //  io.out.bits.uop.ctrl.vecWen := dataModule.i2vIo.vWenOut
  //  //  _ : = dataModule.i2vIo.vdOut
  connectDataModule
  dataModule.regEnables <> VecInit((1 to latency) map (i => regEnable(i)))
}
