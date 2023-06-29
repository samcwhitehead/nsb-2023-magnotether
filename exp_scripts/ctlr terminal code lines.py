
sys.path.insert(0, "/home/scientist/src/panel_comm/panel_com")
from panel_com import PanelCom
userPortString='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A400f6me-if00-port0' # Metal Panel Display Controller
PhidSN=277087
gainOL = 4. # corresponds to 40Hz update rate. for 16Hz update rate (pattern frame rate), set to 1.6
pattern_id = 1
NoPattV, CLVoltage, StatImVoltage = 9.99, 9., 0. # OL voltage is pattern-dependent
Static_dur, OLstim_dur  = 4., 4.
slpT = .05 # for LED panel controller to update
# Assign the LED-PCB to 'PanelCom', under 'ctlr'
ctlr = PanelCom(userport=userPortString)
modex = 0 # open loop
ctlr.SetMode(modex, 0)
time.sleep(slpT)
ctlr.SetGainOffset(gainOL, 0, 0, 0)
time.sleep(slpT)
pattern_id = 1 # 1-based
ctlr.SetPatternID(pattern_id)
time.sleep(slpT)
ctlr.SetGainOffset(gainOL, 0, 0, 0)
time.sleep(slpT)
ctlr.SetPositions(0,subpat) # first frame of pattern, use for static image
ctlr.Start()
time.sleep(5.) # to align pattern change phidgets pattern signal
ctlr.Stop()
time.sleep(slpT)

ctlr.AllOff()
