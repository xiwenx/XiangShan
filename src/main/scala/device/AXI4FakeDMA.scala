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

package device

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
// import difftest.DifftestDMATransaction
import freechips.rocketchip.amba.axi4.{AXI4MasterNode, AXI4MasterPortParameters, AXI4Parameters}
import freechips.rocketchip.diplomacy.AddressSet

class DMAFakeMSHR(implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val enable = Input(Bool())
    val slave = new Bundle {
      val wen   = Input(Bool())
      val addr  = Input(UInt(4.W))
      val rdata = Output(UInt(64.W))
      val wdata = Input(UInt(64.W))
    }
    val master = new Bundle {
      val req = new Bundle {
        val valid = Output(Bool())
        val ready = Input(Bool())
        val is_write = Output(Bool())
        val addr = Output(UInt(64.W))
        val mask = Output(UInt(64.W))
        val data = Output(UInt(512.W))
      }
      val resp = Flipped(ValidIO(UInt(256.W)))
    }
  })

  val state   = Reg(UInt(8.W))
  val address = Reg(UInt(64.W))
  val mask    = Reg(UInt(64.W))
  val data    = Reg(Vec(8, UInt(64.W)))

  val s_idle :: s_read :: s_write :: s_wait_resp_b :: s_wait_resp_r0 :: s_wait_resp_r1 :: Nil = Enum(6)
  when (state === s_read) {
    when (io.master.req.valid && io.master.req.ready) {
      state := s_wait_resp_r0
    }
  }.elsewhen (state === s_write) {
    when (io.master.req.valid && io.master.req.ready) {
      state := s_wait_resp_b
    }
  }.elsewhen (state === s_wait_resp_b) {
    when (io.master.resp.valid) {
      state := s_idle
    }
  }.elsewhen (state === s_wait_resp_r0) {
    when (io.master.resp.valid) {
      state := s_wait_resp_r1
    }
  }.elsewhen (state === s_wait_resp_r1) {
    when (io.master.resp.valid) {
      state := s_idle
    }
  }

  when (io.slave.wen) {
    when (io.slave.addr === 8.U) {
      state := io.slave.wdata
    }.elsewhen(io.slave.addr === 9.U) {
      address := io.slave.wdata
    }.elsewhen(io.slave.addr === 10.U) {
      mask := io.slave.wdata
    }.otherwise {
      data(io.slave.addr) := io.slave.wdata
    }
  }
  io.slave.rdata := Mux(io.slave.addr === 8.U, state,
    Mux(io.slave.addr === 9.U, address,
      Mux(io.slave.addr === 10.U, mask, data(io.slave.addr))))

  io.master.req.valid := io.enable && (state === s_read || state === s_write)
  io.master.req.is_write := state === s_write
  io.master.req.addr := address
  io.master.req.mask := mask
  io.master.req.data := data.asUInt

  when (io.master.resp.valid) {
    when (state === s_wait_resp_r0) {
      for (i <- 0 until 4) {
        data(i) := io.master.resp.bits(64 * i + 63, 64 * i)
      }
    }.elsewhen(state === s_wait_resp_r1) {
      for (i <- 0 until 4) {
        data(i + 4) := io.master.resp.bits(64 * i + 63, 64 * i)
      }
    }
  }

  val last_state = RegNext(state, init=s_idle)
  val read_valid = last_state === s_wait_resp_r1 && state === s_idle
  val write_valid = last_state === s_wait_resp_b && state === s_idle
  // val difftest = Module(new DifftestDMATransaction)
  // difftest.io.clock    := clock
  // difftest.io.coreid   := 0.U
  // difftest.io.valid    := read_valid || write_valid
  // difftest.io.is_write := last_state === s_wait_resp_b
  // difftest.io.address  := address
  // difftest.io.mask     := mask
  // difftest.io.data     := data

  def slave_read(addr: UInt): UInt = {
    io.slave.wen := false.B
    io.slave.addr := addr
    io.slave.rdata
  }
  def slave_write(addr: UInt, data: UInt): Unit = {
    io.slave.wen := true.B
    io.slave.addr := addr
    io.slave.wdata := data
  }
  def has_read_req: Bool = io.master.req.valid && !io.master.req.is_write
  def has_write_req: Bool = io.master.req.valid && io.master.req.is_write
}

class AXI4FakeDMA
(
  address: Seq[AddressSet],
  params: AXI4MasterPortParameters
)(implicit p: Parameters)
  extends AXI4SlaveModule(address, executable = true)
{
  val dma_node = AXI4MasterNode(Seq(params))

  override lazy val module = new AXI4SlaveModuleImp(this) {
    val numInflight = 64
    require(isPow2(numInflight))

    // 0x0 - (0x80 * numInflight)

    // DMACtrl slave READ

    // DMACtrl slave WRITE

    // DMA master
    val (out, dma_edge) = dma_node.out.head
    val dmaReqBytes = 64
    val dmaBeatBytes = dma_edge.slave.beatBytes
    val numBeats = dmaReqBytes / dmaBeatBytes
    val axi_len = numBeats - 1
    def selectByBeatIndex(i: UInt, data: UInt): UInt = {
      data.asTypeOf(Vec(numBeats, UInt((data.getWidth / numBeats).W)))(i)
    }

    // DMA master READ Request
    out.ar.valid := false.B
    out.ar.bits := 0.U.asTypeOf(out.ar.bits.cloneType)
    out.ar.bits.id := 0.U
    out.ar.bits.addr := 0.U
    out.ar.bits.len := axi_len.U
    out.ar.bits.size := log2Ceil(dmaBeatBytes).U
    out.ar.bits.burst := AXI4Parameters.BURST_INCR

    val cnt = RegInit(0.U(64.W))
    cnt := cnt + 1.U

    // DMA master WRIET Request
    when (cnt === 10000.U) {
      assert(out.aw.ready)
    }
    out.aw.valid := (cnt === 10000.U)
    out.aw.bits := 0.U.asTypeOf(out.aw.bits.cloneType)
    out.aw.bits.id := "hc".U
    out.aw.bits.addr := "hf8063040".U
    out.aw.bits.len := "hf".U
    out.aw.bits.size := 3.U
    out.aw.bits.burst := AXI4Parameters.BURST_INCR

    // DMA master READ/WRITE handshake

    // DMA master WRITE DATA
    val w_valid = RegInit(false.B)
    when (out.aw.fire) {
      w_valid := true.B
    }.elsewhen(out.w.fire && out.w.bits.last) {
      w_valid := false.B
    }
    // Only one inflight aw: disable aw.valid when !w.bits.last
    when (w_valid) {
      out.aw.valid := false.B
    }
    val beatCount = RegInit(0.U(log2Ceil(16).W))

    val strb = RegInit("hff".U(32.W))
    when (out.w.fire) {
      strb := Cat(strb(23,0), strb(31,24))
    }

    out.w.valid := w_valid
    out.w.bits := 0.U.asTypeOf(out.w.bits.cloneType)
    out.w.bits.data := "hdeadbeef".U
    out.w.bits.strb := strb
    out.w.bits.last := beatCount === (16 - 1).U
    when (out.w.fire) {
      beatCount := beatCount + 1.U
    }

    // DMA master READ/WRITE Response
    out.r.ready := true.B
    out.b.ready := true.B

    dontTouch(out)

  }
}
