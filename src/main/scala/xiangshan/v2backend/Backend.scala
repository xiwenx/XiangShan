package xiangshan.v2backend

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp}
import utility.PipelineConnect
import xiangshan._
import xiangshan.backend.CtrlBlock
import xiangshan.backend.exu.FenceIO
import xiangshan.backend.fu.{CSRFileIO, FenceToSbuffer, PerfCounterIO}
import xiangshan.backend.rob.RobLsqIO
import xiangshan.frontend.{FtqPtr, FtqRead}
import xiangshan.mem.{LqPtr, LsqEnqIO, SqPtr}
import xiangshan.v2backend.Bundles.{DynInst, MemExuInput, MemExuOutput}

class Backend(val params: BackendParams)(implicit p: Parameters) extends LazyModule
  with HasXSParameter {

  for (exuCfg <- params.allExuParams) {
    val fuConfigs = exuCfg.fuConfigs
    val wbPortConfigs = exuCfg.wbPortConfigs
    val immType = exuCfg.immType
    println(s"exu: ${fuConfigs.map(_.name)}, wb: ${wbPortConfigs}, imm: ${immType}")
    require(wbPortConfigs.collectFirst { case x: IntWB => x }.nonEmpty ==
      fuConfigs.map(_.writeIntRf).reduce(_ || _),
      "int wb port has no priority" )
    require(wbPortConfigs.collectFirst { case x: VecWB => x }.nonEmpty ==
      fuConfigs.map(x => x.writeFpRf || x.writeVecRf).reduce(_ || _),
      "vec wb port has no priority" )
  }

  println(s"Function Unit: Alu(${params.AluCnt}), Brh(${params.BrhCnt}), Jmp(${params.JmpCnt}), " +
    s"Ldu(${params.LduCnt}), Sta(${params.StaCnt}), Std(${params.StdCnt})")

  val ctrlBlock = LazyModule(new CtrlBlock(params))
  val intScheduler = params.intSchdParams.map(x => LazyModule(new Scheduler(x)))
//  val vfScheduler = params.vfSchdParams.map(x => LazyModule(new Scheduler(x)))
  val memScheduler = params.memSchdParams.map(x => LazyModule(new Scheduler(x)))
  val dataPath = LazyModule(new DataPath(params))
  val intExuBlock = params.intSchdParams.map(x => LazyModule(new ExuBlock(x)))
//  val vfExuBlock = params.vfSchdParams.map(x => LazyModule(new ExuBlock(x)))

  lazy val module = new BackendImp(this)
}

class BackendImp(override val wrapper: Backend)(implicit p: Parameters) extends LazyModuleImp(wrapper) {
  implicit private val params = wrapper.params
  val io = IO(new BackendIO()(p, wrapper.params))

  private val ctrlBlock = wrapper.ctrlBlock.module
  private val intScheduler = wrapper.intScheduler.get.module
//  private val vfScheduler = wrapper.vfScheduler.module
  private val memScheduler = wrapper.memScheduler.get.module
  private val dataPath = wrapper.dataPath.module
  private val intExuBlock = wrapper.intExuBlock.get.module
//  private val fpExuBlock = wrapper.vfExuBlock.get.module
  private val wbDataPath = Module(new WbDataPath(params))

  ctrlBlock.io.fromTop.hartId := io.fromTop.hartId
  ctrlBlock.io.frontend <> io.frontend
  ctrlBlock.io.fromWB.wbData <> wbDataPath.io.toCtrlBlock.writeback
  ctrlBlock.io.fromMem.stIn <> io.mem.stIn
  ctrlBlock.io.fromMem.violation <> io.mem.memoryViolation
  ctrlBlock.io.csrCtrl <> intExuBlock.io.csrio.get.customCtrl
  ctrlBlock.io.robio.csr.intrBitSet := intExuBlock.io.csrio.get.interrupt
  ctrlBlock.io.robio.csr.trapTarget := intExuBlock.io.csrio.get.trapTarget
  ctrlBlock.io.robio.csr.isXRet := intExuBlock.io.csrio.get.isXRet
  ctrlBlock.io.robio.csr.wfiEvent := intExuBlock.io.csrio.get.wfi_event
  ctrlBlock.io.robio.lsq <> io.mem.robLsqIO

  intScheduler.io.fromTop.hartId := io.fromTop.hartId
  intScheduler.io.fromCtrlBlock.flush := ctrlBlock.io.toIssueBlock.flush
  intScheduler.io.fromCtrlBlock.pcVec := ctrlBlock.io.toIssueBlock.pcVec
  intScheduler.io.fromCtrlBlock.targetVec := ctrlBlock.io.toIssueBlock.targetVec
  intScheduler.io.fromDispatch.allocPregs <> ctrlBlock.io.toIssueBlock.allocPregs
  intScheduler.io.fromDispatch.uops <> ctrlBlock.io.toIssueBlock.intUops
  intScheduler.io.writeback := wbDataPath.io.toIntPreg.map(_.toWakeUpBundle)

  memScheduler.io.fromTop.hartId := io.fromTop.hartId
  memScheduler.io.fromCtrlBlock.flush := ctrlBlock.io.toIssueBlock.flush
  memScheduler.io.fromDispatch.allocPregs <> ctrlBlock.io.toIssueBlock.allocPregs
  memScheduler.io.fromDispatch.uops <> ctrlBlock.io.toIssueBlock.memUops
  memScheduler.io.writeback := wbDataPath.io.toIntPreg.map(_.toWakeUpBundle) ++ wbDataPath.io.toVfPreg.map(_.toWakeUpBundle)
  memScheduler.io.fromMem.get.scommit := ctrlBlock.io.robio.lsq.scommit
  memScheduler.io.fromMem.get.lcommit := ctrlBlock.io.robio.lsq.lcommit
  memScheduler.io.fromMem.get.sqCancelCnt := io.mem.sqCancelCnt
  memScheduler.io.fromMem.get.lqCancelCnt := io.mem.lqCancelCnt
  memScheduler.io.fromMem.get.stIssuePtr := io.mem.stIssuePtr
  memScheduler.io.fromMem.get.memWaitUpdateReq.staIssue.zip(io.mem.stIn).foreach { case (sink, source) =>
    sink.valid := source.valid
    sink.bits.uop := 0.U.asTypeOf(sink.bits.uop)
    sink.bits.uop.robIdx := source.bits.robIdx
  }

//  vfScheduler.io.fromTop.hartId := io.fromTop.hartId
//  vfScheduler.io.fromCtrlBlock.flush := ctrlBlock.io.redirect
//  vfScheduler.io.fromDispatch.allocPregs <> ctrlBlock.io.toIssueBlock.allocPregs
//  vfScheduler.io.fromDispatch.uops <> ctrlBlock.io.toIssueBlock.uops
//  vfScheduler.io.writeback := wbDataPath.io.toVfPreg.map(_.toWakeUpBundle)

  dataPath.io.flush := ctrlBlock.io.toDataPath.flush

  for (i <- 0 until dataPath.io.fromIntIQ.length) {
    for (j <- 0 until dataPath.io.fromIntIQ(i).length) {
      PipelineConnect(intScheduler.io.toDataPath(i)(j), dataPath.io.fromIntIQ(i)(j), dataPath.io.fromIntIQ(i)(j).fire,
        intScheduler.io.toDataPath(i)(j).bits.common.robIdx.needFlush(ctrlBlock.io.redirect))
    }
  }

  dataPath.io.fromVfIQ <> 0.U.asTypeOf(dataPath.io.fromVfIQ)
  dataPath.io.fromMemIQ <> memScheduler.io.toDataPath

  println(s"[Backend] wbDataPath.io.toIntPreg: ${wbDataPath.io.toIntPreg.size}, dataPath.io.fromIntWb: ${dataPath.io.fromIntWb.size}")
  println(s"[Backend] wbDataPath.io.toVfPreg: ${wbDataPath.io.toVfPreg.size}, dataPath.io.fromFpWb: ${dataPath.io.fromFpWb.size}")
  dataPath.io.fromIntWb := wbDataPath.io.toIntPreg
  dataPath.io.fromFpWb := wbDataPath.io.toVfPreg
  dataPath.io.debugIntRat := ctrlBlock.io.debug_int_rat
  dataPath.io.debugFpRat := ctrlBlock.io.debug_fp_rat
  dataPath.io.debugVecRat := ctrlBlock.io.debug_vec_rat

  intExuBlock.io.flush := ctrlBlock.io.toExuBlock.flush
  for (i <- 0 until intExuBlock.io.in.length) {
    for (j <- 0 until intExuBlock.io.in(i).length) {
      PipelineConnect(dataPath.io.toIntExu(i)(j), intExuBlock.io.in(i)(j), intExuBlock.io.in(i)(j).fire,
        dataPath.io.toIntExu(i)(j).bits.robIdx.needFlush(ctrlBlock.io.redirect))
    }
  }

  private val csrio = intExuBlock.io.csrio.get
  csrio.hartId := io.fromTop.hartId
  csrio.perf.retiredInstr <> ctrlBlock.io.robio.csr.perfinfo.retiredInstr
  csrio.perf.ctrlInfo <> ctrlBlock.io.perfInfo.ctrlInfo
  csrio.perf.perfEventsCtrl <> ctrlBlock.getPerf
  csrio.fpu.fflags := ctrlBlock.io.robio.csr.fflags
  csrio.fpu.isIllegal := false.B // Todo: remove it
  csrio.fpu.dirty_fs := ctrlBlock.io.robio.csr.dirty_fs
  csrio.vpu <> 0.U.asTypeOf(csrio.vpu) // Todo
  csrio.exception := ctrlBlock.io.robio.exception
  csrio.memExceptionVAddr := io.mem.exceptionVAddr
  csrio.externalInterrupt := io.fromTop.externalInterrupt
  csrio.distributedUpdate(0) := io.mem.csrDistributedUpdate
  csrio.distributedUpdate(1) := io.frontendCsrDistributedUpdate
  csrio.perf <> io.perf
  private val fenceio = intExuBlock.io.fenceio.get
  fenceio.disableSfence := csrio.disableSfence
  io.fenceio <> fenceio

//  fpExuBlock.io.flush := ctrlBlock.io.toExuBlock.flush
//  fpExuBlock.io.in := dataPath.io.toFpExu
//  fpExuBlock.io.frm.get := intExuBlock.io.csrio.get.fpu.frm

  wbDataPath.io.flush := ctrlBlock.io.redirect
  wbDataPath.io.fromIntExu <> intExuBlock.io.out
  wbDataPath.io.fromVfExu <> 0.U.asTypeOf(wbDataPath.io.fromVfExu) // fpExuBlock.io.out
  wbDataPath.io.fromMemExu.flatten.zip(io.mem.writeBack).foreach { case (sink, source) =>
    sink.valid := source.valid
    source.ready := sink.ready
    sink.bits.data   := source.bits.data
    sink.bits.pdest  := source.bits.uop.pdest
    sink.bits.robIdx := source.bits.uop.robIdx
    sink.bits.intWen.foreach(_ := source.bits.uop.rfWen)
    sink.bits.fpWen.foreach(_ := source.bits.uop.fpWen)
    sink.bits.vecWen.foreach(_ := source.bits.uop.vecWen)
    sink.bits.exceptionVec.foreach(_ := source.bits.uop.exceptionVec)
    sink.bits.flushPipe.foreach(_ := source.bits.uop.flushPipe)
    sink.bits.replay.foreach(_ := source.bits.uop.replayInst)
    sink.bits.debug := 0.U.asTypeOf(sink.bits.debug)
    sink.bits.debugInfo := 0.U.asTypeOf(sink.bits.debugInfo)
    sink.bits.lqIdx.foreach(_ := source.bits.uop.lqIdx)
    sink.bits.sqIdx.foreach(_ := source.bits.uop.sqIdx)
    sink.bits.ftqIdx.foreach(_ := source.bits.uop.ftqPtr)
    sink.bits.ftqOffset.foreach(_ := source.bits.uop.ftqOffset)
  }

  // to mem
  io.mem.redirect := ctrlBlock.io.redirect
  io.mem.issueUops.zip(dataPath.io.toMemExu.flatten).foreach { case (sink, source) =>
    sink.valid := source.valid
    source.ready := sink.ready
    sink.bits.uop := 0.U.asTypeOf(sink.bits.uop)
    sink.bits.src := 0.U.asTypeOf(sink.bits.src)
    sink.bits.src.zip(source.bits.src).foreach { case (l, r) => l := r}
    sink.bits.uop.fuType    := source.bits.fuType
    sink.bits.uop.fuOpType  := source.bits.fuOpType
    sink.bits.uop.imm       := source.bits.imm
    sink.bits.uop.robIdx    := source.bits.robIdx
    sink.bits.uop.pdest     := source.bits.pdest
    sink.bits.uop.rfWen     := source.bits.rfWen.getOrElse(false.B)
    sink.bits.uop.fpWen     := source.bits.fpWen.getOrElse(false.B)
    sink.bits.uop.vecWen    := source.bits.vecWen.getOrElse(false.B)
    sink.bits.uop.flushPipe := source.bits.flushPipe.getOrElse(false.B)
    sink.bits.uop.pc        := source.bits.pc.getOrElse(0.U)
    sink.bits.uop.lqIdx     := source.bits.lqIdx.getOrElse(0.U.asTypeOf(new LqPtr))
    sink.bits.uop.sqIdx     := source.bits.sqIdx.getOrElse(0.U.asTypeOf(new SqPtr))
    sink.bits.uop.ftqPtr    := source.bits.ftqIdx.getOrElse(0.U.asTypeOf(new FtqPtr))
    sink.bits.uop.ftqOffset := source.bits.ftqOffset.getOrElse(0.U)
  }
  io.mem.loadFastMatch := memScheduler.io.toMem.get.loadFastMatch.map(_.fastMatch)
  io.mem.loadFastImm := memScheduler.io.toMem.get.loadFastMatch.map(_.fastImm)
  io.mem.tlbCsr := csrio.tlb
  io.mem.csrCtrl := csrio.customCtrl
  io.mem.sfence := fenceio.sfence
  io.mem.isStoreException := CommitType.lsInstIsStore(ctrlBlock.io.robio.exception.bits.commitType)
  require(io.mem.loadPcRead.size == params.LduCnt)
  io.mem.loadPcRead.zipWithIndex.foreach { case (loadPcRead, i) =>
    loadPcRead.data := ctrlBlock.io.memLdPcRead(i).data
    ctrlBlock.io.memLdPcRead(i).ptr := loadPcRead.ptr
    ctrlBlock.io.memLdPcRead(i).offset := loadPcRead.offset
  }
  // mem io
  io.mem.lsqEnqIO <> memScheduler.io.memIO.get.lsqEnqIO
  io.mem.robLsqIO <> ctrlBlock.io.robio.lsq
  io.mem.toSbuffer <> fenceio.sbuffer
  io.mem.rsFeedBack <> memScheduler.io.memIO.get.feedbackIO

  io.frontendSfence := fenceio.sfence
  io.frontendTlbCsr := csrio.tlb
  io.frontendCsrCtrl := csrio.customCtrl

  io.tlb <> csrio.tlb

  io.csrCustomCtrl := csrio.customCtrl

  dontTouch(memScheduler.io)
  dontTouch(io.mem)
  dontTouch(dataPath.io.toMemExu)
  dontTouch(wbDataPath.io.fromMemExu)
}

class BackendMemIO(implicit p: Parameters, params: BackendParams) extends XSBundle {
  val rsParameters = p.alter((site, here, up) => {
    case XSCoreParamsKey => up(XSCoreParamsKey).copy(
      IssQueSize = 16 // Todo
    )
  })
  // In/Out // Todo: split it into one-direction bundle
  val lsqEnqIO = Flipped(new LsqEnqIO)
  val robLsqIO = new RobLsqIO
  val toSbuffer = new FenceToSbuffer
  val rsFeedBack = Vec(params.StaCnt, Flipped(new MemRSFeedbackIO()(rsParameters)))
  val loadPcRead = Vec(params.LduCnt, Flipped(new FtqRead(UInt(VAddrBits.W))))

  // Input
  val writeBack = Vec(params.LduCnt + params.StaCnt * 2, Flipped(DecoupledIO(new MemExuOutput())))

  val s3_delayed_load_error = Input(Vec(LoadPipelineWidth, Bool()))
  val stIn = Input(Vec(params.StaCnt, ValidIO(new DynInst())))
  val memoryViolation = Flipped(ValidIO(new Redirect))
  val exceptionVAddr = Input(UInt(VAddrBits.W))
  val sqDeq = Input(UInt(params.StaCnt.W))

  val lqCancelCnt = Input(UInt(log2Up(LoadQueueSize + 1).W))
  val sqCancelCnt = Input(UInt(log2Up(StoreQueueSize + 1).W))

  val otherFastWakeup = Flipped(Vec(params.LduCnt + 2 * params.StaCnt, ValidIO(new DynInst)))
  val stIssuePtr = Input(new SqPtr())

  val csrDistributedUpdate = Flipped(new DistributedCSRUpdateReq)

  // Output
  val redirect = ValidIO(new Redirect)   // rob flush MemBlock
  val issueUops = Vec(params.LduCnt + 2 * params.StaCnt, DecoupledIO(new MemExuInput()))
  val loadFastMatch = Vec(params.LduCnt, Output(UInt(params.LduCnt.W)))
  val loadFastImm   = Vec(params.LduCnt, Output(UInt(12.W))) // Imm_I

  val tlbCsr = Output(new TlbCsrBundle)
  val csrCtrl = Output(new CustomCSRCtrlIO)
  val sfence = Output(new SfenceBundle)
  val isStoreException = Output(Bool())
}

class BackendIO(implicit p: Parameters, params: BackendParams) extends XSBundle {
  val fromTop = new Bundle {
    val hartId = Input(UInt(8.W))
    val externalInterrupt = new ExternalInterruptIO
  }

  val toTop = new Bundle {
    val cpuHalted = Output(Bool())
  }

  val fenceio = new FenceIO
  // Todo: merge these bundles into BackendFrontendIO
  val frontend = Flipped(new FrontendToCtrlIO)
  val frontendSfence = Output(new SfenceBundle)
  val frontendCsrCtrl = Output(new CustomCSRCtrlIO)
  val frontendTlbCsr = Output(new TlbCsrBundle)
  // distributed csr write
  val frontendCsrDistributedUpdate = Flipped(new DistributedCSRUpdateReq)

  val mem = new BackendMemIO

  val perf = Input(new PerfCounterIO)

  val tlb = Output(new TlbCsrBundle)

  val csrCustomCtrl = Output(new CustomCSRCtrlIO)
}