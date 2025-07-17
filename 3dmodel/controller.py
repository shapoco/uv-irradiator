import cadquery as cq
import math

MIL100 = 2.54
GENERIC_GAP = 0.5
NARROW_GAP = 0.2
GENERIC_CHAMFER = 0.5

T_BOARD = 1

class TactSw:
    BODY_W = 6.2
    BODY_H = 6.2
    BODY_T= 4
    TOTAL_T = 5.1
    BUTTON_D = 3.5
    
    def __init__(self, x, y):
        self.x = x
        self.y = y

class SwKey:
    TOP_T = 4
    FLANGE_T = 1
    FLANGE_W = 1
    ENGRAVE_DEPTH = 0.75
    
    def __init__(self, sw : TactSw,  w, h, z_gap, text):
        self.w = w
        self.h = h
        self.z_gap = z_gap
        self.sw = sw
        self.text = text

        # キートップ原型
        solid = (
            cq.Workplane("XY")
            .box(self.w, self.h, SwKey.TOP_T, centered=(True, True, False))
        )
        
        # キートップを斜めにカット
        verts = [
            (-99, 0),
            (0, 0),
            (0.5, SwKey.TOP_T),
            (99, SwKey.TOP_T),
            (99, SwKey.TOP_T+1),
            (-99, SwKey.TOP_T +1),
        ]
        cutter = (
            cq.Workplane("XZ")
            .polyline(verts)
            .close()
            .extrude(99, both=True)
        )
        solid = solid.cut(cutter.translate((-self.w / 2, 0, 0)))
        solid = solid.cut(cutter.mirror("YZ").translate((self.w / 2, 0, 0)))
        cutter = cutter.rotate((0, 0, 0), (0, 0, 1), 90)
        solid = solid.cut(cutter.translate((0, -self.h / 2, 0)))
        solid = solid.cut(cutter.mirror("XZ").translate((0, self.h / 2, 0)))
        
        # 面取り
        solid = (
            solid
            .edges("not <<Z")
            .fillet(GENERIC_CHAMFER)
            .faces(">Z")
            .workplane(origin=(0,0,0))
            .text(self.text, 4.5, -SwKey.ENGRAVE_DEPTH, kind="bold", halign="center", valign="center")
        )
        
        # フランジの追加
        solid = solid.union(
            cq.Workplane("XY")
            .box(self.w + SwKey.FLANGE_W * 2, self.h + SwKey.FLANGE_W * 2, SwKey.FLANGE_T, centered=(True, True, False))
            .translate((0, 0, -SwKey.FLANGE_T))
        )
        
        # 裏側の彫り込み
        solid = solid.cut(
            cq.Workplane("XY")
            .box(
                TactSw.BODY_W + GENERIC_GAP,
                TactSw.BODY_H + GENERIC_GAP,
                TactSw.TOTAL_T + GENERIC_GAP - 0.5,
                centered=(True, True, False),
            )
            .translate((0, 0, -self.z_gap))
        )
        
        # ボタンの軸がはまる穴
        solid = solid.cut(
            cq.Workplane("XY")
            .cylinder(
                TactSw.TOTAL_T + GENERIC_GAP + NARROW_GAP, 
                TactSw.BUTTON_D / 2 + GENERIC_GAP,
                centered=(True, True, False),
            )
            .translate((0, 0, -self.z_gap))
        )
        
        self.solid = solid.translate((sw.x, sw.y, 0))
        
    
    def modify_panel(self, panel, offset):
        cutter = (
            cq.Workplane("XY")
            .box(self.w + GENERIC_GAP, self.h + GENERIC_GAP, 50, centered=(True, True, False))
            .edges("|Z")
            .fillet(GENERIC_CHAMFER)
            .translate((self.sw.x, self.sw.y, 0))
            .translate(offset)
        )
        #show_object(cutter)
        panel = panel.cut(cutter)
        return panel

class LED:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Indicator():
    T_TRANSPARENT = 0.5
    ENGRAVE_DEPTH = 0.75
    T_PLATE = T_TRANSPARENT + ENGRAVE_DEPTH
    WALL_THICKNESS = 0.5
    PANEL_FILLET = 1
    PANEL_CHAMFER = 0.5
    PANEL_NOTCH = 0.3
    WALL_Z_MARGIN = 1
    
    
    def __init__(self, led : LED, w, h, panel_thickness, z_gap, text):
        self.w = w
        self.h = h
        self.panel_thickness = panel_thickness
        self.z_gap = z_gap
        self.led = led
        self.text = text
    
    def modify_panel(self, panel, offset):
        wall = (
            cq.Workplane("XY")
            .box(self.w + Indicator.WALL_THICKNESS*2, self.h + Indicator.WALL_THICKNESS *2, self.z_gap - Indicator.WALL_Z_MARGIN, centered=(True, True, False))
        )
        wall = wall.cut (
            cq.Workplane("XY")
            .box(self.w, self.h, self.z_gap, centered=(True, True, False))
        )
        wall = (
            wall.translate((self.led.x, self.led.y, Indicator.WALL_Z_MARGIN))
            .translate(offset)
        )
        panel = panel.union(wall)
        
        cutter = (
            cq.Workplane("XY")
            .box(self.w , self.h, self.panel_thickness, centered=(True, True, False))
            .faces(">Z")
            .workplane(origin=(0,0,0))
            .rect(self.w -Indicator.PANEL_NOTCH*2, self.h-Indicator.PANEL_NOTCH*2)
            .extrude(Indicator.T_TRANSPARENT)
            .translate((0, 0, -Indicator.T_TRANSPARENT))
        )
        panel = panel.cut(cutter.translate((self.led.x, self.led.y, self.z_gap)).translate(offset))

        return panel

    def make_plate(self):
        GAP = 0.2
        solid = (
            cq.Workplane("XY")
            .box(self.w - GAP, self.h - GAP, Indicator.ENGRAVE_DEPTH, centered=(True, True, False))
            .faces(">Z")
            .workplane(origin=(0,0,0))
            .rect(self.w -Indicator.PANEL_NOTCH*2-GAP, self.h-Indicator.PANEL_NOTCH*2-GAP)
            .extrude(Indicator.T_TRANSPARENT)
            .faces(">Z")
            .workplane(origin=(0,0,0))
            .text(self.text, 4, -Indicator.ENGRAVE_DEPTH, kind="bold" , halign="center", valign="center")
        )
        
        return solid
    
class SidePinHeader:
    L_PIN = 7
    W_PIN = 0.75

    def __init__(self, num_pin_w, num_pin_h, offset):
        self.num_pin_w = num_pin_w
        self.num_pin_h = num_pin_h
        self.offset = offset
        self.w = (num_pin_w + 1) * MIL100
        self.h = (num_pin_h + 1) * MIL100
        self.cutter = (
            cq.Workplane("XY")
            .box(self.h, 20, self.w, centered=(False, False, False))
            .translate((-MIL100, 0, -0.5 * MIL100 + SidePinHeader.W_PIN / 2))
            .translate(self.offset)
        )


class SSD1906:
    W = 27.35
    H = 27.8
    
    class Screen:
        W = 26.5
        H = 14.5
        T = 1.5
        X_OFFSET = (27.35 - W) / 2
        Y_OFFSET = 5

    T_TOTAL = 3
    T_BOARD = T_TOTAL - Screen.T
    
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def modify_panel(self, panel, offset):
        cutter = (
            cq.Workplane("XY")
            .box(SSD1906.Screen.W, SSD1906.Screen.H, 100, centered=(False, False, False))
            .edges("|Z")
            .fillet(GENERIC_CHAMFER)
            .translate((self.x + SSD1906.Screen.X_OFFSET, self.y + SSD1906.Screen.Y_OFFSET, 0))
            .translate(offset)
        )
        #show_object(cutter)
        panel = panel.cut(cutter)
        
        return panel

class RasPico:
    T_BOARD = 1
    W = 21.5
    L = 51.5
    
    W_CONN = 7.6
    L_CONN = 4
    T_CONN = 2.5
    
    SOLID_TEMPLATE = (
        cq.Workplane("XY")
        .box(W, L, T_BOARD, centered=(True, False, False))
    )
    SOLID_TEMPLATE = SOLID_TEMPLATE.union(
        cq.Workplane("XY")
        .box(W_CONN, L_CONN, T_CONN, centered=(True, False, False))
        .translate((0, 0, T_BOARD))
    )

class MainBoard:
    W = 22 * MIL100
    H = 18 * MIL100
    
    T_FRONT = 14
    T_BACK = 2
    
    CN1 = SidePinHeader(2, 8, (2 * MIL100, H, 0))
    CN2 = SidePinHeader(1, 2, (11 * MIL100, H, 0))
    
    T_TOTAL = T_BOARD + T_FRONT + T_BACK
    H_INCLUDE_PINS = H + CN1.L_PIN

class Volume:
    BODY_DIAMETER = 17
    BODY_THICKNESS_INCLUDING_SW_PINS = 15.1
        
    # 軸の中心から中央のピンの先端までの距離
    PIN_RADIUS = 16
    
    SHAFT_HOLE_DIAMETER = 7
    
    SHAFT_D = 6
    SHAFT_L = 20
    
    LOCK_HOLE_W = 1.5
    LOCK_HOLE_H = 3
    LOCK_HOLE_X_OFFSET = 7.8
    
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle
        solid = (
            cq.Workplane("XY")
            .cylinder(Volume.BODY_THICKNESS_INCLUDING_SW_PINS, Volume.BODY_DIAMETER / 2, centered=(True, True, False))
            .translate((x, y, -Volume.BODY_THICKNESS_INCLUDING_SW_PINS))
        )
        solid = solid.union(
            cq.Workplane("XY")
            .cylinder(Volume.SHAFT_L, Volume.SHAFT_D / 2, centered=(True, True, False))
            .faces(">Z")
            .chamfer(GENERIC_CHAMFER)
            .translate((x, y, 0))
        )
        self.solid = solid
        
    def modify_panel(self, panel, offset):
        cutter = (
            cq.Workplane("XY")
            .cylinder(100, (Volume.SHAFT_HOLE_DIAMETER + GENERIC_GAP) / 2)
        )
        cutter = cutter.union(
            cq.Workplane("XY")
            .box(Volume.LOCK_HOLE_W + GENERIC_GAP, Volume.LOCK_HOLE_H + GENERIC_GAP, 100)
            .edges("|Z")
            .fillet(0.1)
            .translate((-Volume.LOCK_HOLE_X_OFFSET, 0, 0))
        )
        cutter = (
            cutter
            .rotate((0, 0, 0), (0, 0, 1), self.angle)
            .translate((self.x , self.y , 0))
            .translate(offset)
        )
        #show_object(cutter)
        panel = panel.cut(cutter)
        
        return panel

class MJ_10A:
    SCREW_DIAMETER = 12.5
    FRONT_DIAMETER = 16
    CUTTER_LENGTH = 20
    
    def __init__(self, offset=(0, 0, 0)):
        self.hole_cutter = (
            cq.Workplane("XY")
            .cylinder(MJ_10A.CUTTER_LENGTH, (MJ_10A.SCREW_DIAMETER + GENERIC_GAP) / 2, centered=(True, True, False))
            .translate((0, 0, -MJ_10A.CUTTER_LENGTH))
            .translate(offset)
        )
        self.offset = offset
        

    def rotate(self, axis_start, axis_end, angle):
        self.hole_cutter = self.hole_cutter.rotate(axis_start, axis_end, angle)
        #self.offset = cq.rotate(self.offset, axis_start, axis_end, angle)
        return self
    
    def translate(self, offset):
        self.hole_cutter = self.hole_cutter.translate(offset)
        #self.offset = cq.translate(self.offset, offset) 
        return self
    
    def cut_panel(self, panel):
        panel = panel.cut(self.hole_cutter)
        return panel

class IntfBoard:
    W = MainBoard.W
    H = 8 * MIL100

    SW_INTERVAL = 5 * MIL100

    LED1 = LED(2.5 * MIL100, 3 * MIL100 -3)
    LED2 = LED(LED1.x, 3 * MIL100 + 3)

    SW1 = TactSw(7.5 * MIL100, 3 * MIL100)
    SW2 = TactSw(SW1.x + SW_INTERVAL, SW1.y)
    SW3 = TactSw(SW2.x + SW_INTERVAL, SW1.y)

    OLED = SSD1906(25.7, 15.9)

    H_INCLUDE_OLED = OLED.y + OLED.H


PANEL_X_MARGIN = 1
PANEL_Y_MARGIN = 1
INTF_BOARD_FRONT_GAP = SSD1906.T_TOTAL + GENERIC_GAP
INTF_BOARD_2D_OFFSET = (PANEL_X_MARGIN, PANEL_Y_MARGIN + 1.5, 0)
INTF_BOARD_3D_OFFSET = (INTF_BOARD_2D_OFFSET[0], INTF_BOARD_2D_OFFSET[1], -INTF_BOARD_FRONT_GAP - T_BOARD)
MAIN_BOARD_FRONT_GAP = INTF_BOARD_FRONT_GAP + T_BOARD  + MainBoard.T_FRONT
MAIN_BOARD_2D_OFFSET = (PANEL_X_MARGIN, PANEL_Y_MARGIN, 0)
MAIN_BOARD_3D_OFFSET = (MAIN_BOARD_2D_OFFSET[0], MAIN_BOARD_2D_OFFSET[1], -MAIN_BOARD_FRONT_GAP - T_BOARD)
INNER_TALL = int(math.ceil(-MAIN_BOARD_3D_OFFSET[2] + MainBoard.T_BACK))

VOL_MARGIN = 2

BOARD_LOCK_W = 5

# パネル厚み
T_PANEL = 2

# DCジャックの取り付け部の寸法
DC_JACK_MOUNT_W = 18 + T_PANEL * 2
DC_JACK_MOUNT_H = 18

H_FLANGE = 4
T_FLANGE = 1

# パネルサイズ算出
PANEL_W = int(math.ceil(MAIN_BOARD_2D_OFFSET[0] + MainBoard.W + DC_JACK_MOUNT_W + PANEL_X_MARGIN))
PANEL_H = int(math.ceil(MAIN_BOARD_3D_OFFSET[1] + MainBoard.H + PANEL_Y_MARGIN))
#PANEL_H = int(math.ceil(INTF_BOARD_2D_OFFSET[1] + IntfBoard.H_INCLUDE_OLED + PANEL_Y_MARGIN))

print(f"INTF_BOARD_FRONT_GAP: {INTF_BOARD_FRONT_GAP}")
print(f"MAIN_BOARD_FRONT_GAP: {MAIN_BOARD_FRONT_GAP}")
print(f"PANEL_W: {PANEL_W}")
print(f"PANEL_H: {PANEL_H}")
print(f"INNER_HEIGHT: {INNER_TALL}")

# 裏ブラ固定用穴の寸法
CASE_LOCK_W = 5
CASE_LOCK_H = 2
CASE_LOCK_DEPTH = 0.5

# 裏蓋固定用の爪穴の位置
offset_y = 10 - CASE_LOCK_W/2 
offset_z  =T_PANEL-H_FLANGE/2
CASE_LOCK_POINTS = [
    ((0, offset_y, offset_z), False),
    ((0, PANEL_H-offset_y, offset_z), False),
    ((PANEL_W, offset_y, offset_z), True),
    ((PANEL_W, PANEL_H-offset_y, offset_z), True),
]

# DCジャックの穴の位置
DC_JACK_HOLE_OFFSET = (PANEL_W - DC_JACK_MOUNT_W/2, PANEL_H -2, -DC_JACK_MOUNT_H/2)

PICO_CONN_HOLE_OFFSET = (
    0,
    MAIN_BOARD_3D_OFFSET[1] + 4.5 * MIL100,
    MAIN_BOARD_3D_OFFSET[2] + T_BOARD + RasPico.T_BOARD + RasPico.T_CONN / 2,
)

# 基板を押さえつける柱の生成
def make_board_supporter(w,h , tall):
    return (
        cq.Workplane("XY")
        .box(w, h, tall - NARROW_GAP, centered=False)
    )

# 基板をロックする部分の生成
def make_board_lock(tall):
    T = 1
    CHAMFER = 0.75
    LOCK_SIZE = 0.75
    MAX_TALL = 10
    
    lock_tall = min(MAX_TALL, tall) + NARROW_GAP
    verts = [
        (CHAMFER, 0),
        (0, CHAMFER),
        (0, lock_tall),
        (LOCK_SIZE, lock_tall ),
        (LOCK_SIZE, lock_tall+0.5),
        (0, lock_tall+0.5 + LOCK_SIZE),
        (-T, lock_tall+0.5 + LOCK_SIZE),
        (-T, CHAMFER),
        (-T-CHAMFER, 0),
    ]
    solid = (
        cq.Workplane("YZ")
        .polyline(verts)
        .close()
        .extrude(BOARD_LOCK_W)
        .faces(">Y")
        .edges("|Z")
        .chamfer(LOCK_SIZE)
    )
    return solid

class FrontPanel:
    
    # キーのサイズ
    KEY_WIDTH = 10
    KEY_HEIGHT = 10
    
    # キー
    KEY1 = SwKey(IntfBoard.SW1, KEY_WIDTH, KEY_HEIGHT, INTF_BOARD_FRONT_GAP, "FN")
    KEY2 = SwKey(IntfBoard.SW2, KEY_WIDTH, KEY_HEIGHT, INTF_BOARD_FRONT_GAP, "▼")
    KEY3 = SwKey(IntfBoard.SW3, KEY_WIDTH, KEY_HEIGHT, INTF_BOARD_FRONT_GAP, "OK")

    # インジケータ
    IND1 = Indicator(IntfBoard.LED1, 12, 5, T_PANEL, INTF_BOARD_FRONT_GAP, "OUT")
    IND2 = Indicator(IntfBoard.LED2, 12, 5, T_PANEL, INTF_BOARD_FRONT_GAP, "ERR")

    # ボリューム
    VR1 = Volume(
        PANEL_W - Volume.BODY_DIAMETER / 2 - VOL_MARGIN,
        VOL_MARGIN + Volume.BODY_DIAMETER / 2,
        180
    )

    DC_JACK = (
        MJ_10A().rotate((0,0,0),(1,0,0),-90)
        .translate(DC_JACK_HOLE_OFFSET)
    )

    solid = (
            cq.Workplane("XY")
            .box(PANEL_W, PANEL_H, T_PANEL, centered=False)
        )
    
    intf_part_offset = (INTF_BOARD_3D_OFFSET[0], INTF_BOARD_3D_OFFSET[1], INTF_BOARD_3D_OFFSET[2]+ T_BOARD)
    solid = KEY1.modify_panel(solid, intf_part_offset)
    solid = KEY2.modify_panel(solid, intf_part_offset)
    solid = KEY3.modify_panel(solid, intf_part_offset)
    solid = IND1.modify_panel(solid, intf_part_offset)
    solid = IND2.modify_panel(solid, intf_part_offset)
    solid = IntfBoard.OLED.modify_panel(solid, intf_part_offset)
    solid = VR1.modify_panel(solid, (0,0,0))

    support_w = 4
    support_h = 2
    support = (
        make_board_supporter(support_w,support_h, INTF_BOARD_FRONT_GAP)
        .mirror("XY")
        .translate(INTF_BOARD_2D_OFFSET)    
    )
    solid = solid.union(support)
    solid = solid.union(support.translate((
        IntfBoard.OLED.x - support_w - 1,
        IntfBoard.H - support_h,
        0,
    )))
    solid = solid.union(support.translate((IntfBoard.W - support_w, IntfBoard.SW3.y, 0)))
    solid = solid.union(support.translate((
        IntfBoard.OLED.x+ SSD1906.W*1/5 - support_w/2, 
        IntfBoard.OLED.y + SSD1906.H - support_h, 
        SSD1906.T_BOARD,
    )))
    solid = solid.union(support.translate((
        IntfBoard.OLED.x+ SSD1906.W*4/5 - support_w/2, 
        IntfBoard.OLED.y + SSD1906.H - support_h, 
        SSD1906.T_BOARD,
    )))
    
    lock_tall = INTF_BOARD_FRONT_GAP + T_BOARD
    lock_template = (
        make_board_lock(lock_tall)
        .mirror("XY")
    )
    locks = lock_template.translate((
        IntfBoard.W *3 /4- BOARD_LOCK_W/2,
        0,
        0,
    ))
    locks = locks.union(lock_template.translate((
        IntfBoard.W*1/4 - BOARD_LOCK_W  /2,
        0,
        0,
    )))
    locks = locks.union(lock_template.mirror("XZ").translate((
        1,
        IntfBoard.H,
        0,
    )))
    locks = locks.union(lock_template.rotate((0,0,0),(0,0,1),90).translate((
        IntfBoard.W,
        IntfBoard.H - BOARD_LOCK_W,
        0,
    )))
    lock_tall = INTF_BOARD_FRONT_GAP
    lock_template = (
        make_board_lock(lock_tall)
        .mirror("XY")
    )
    locks = locks.union(lock_template.rotate((0,0,0),(0,0,1),-90).translate((
        IntfBoard.OLED.x,
        IntfBoard.OLED.y + SSD1906.H,
        0,
    )))
    locks = locks.union(lock_template.rotate((0,0,0),(0,0,1),90).translate((
        IntfBoard.OLED.x + SSD1906.W,
        IntfBoard.OLED.y + SSD1906.H - BOARD_LOCK_W,
        0,
    )))
    
    solid = solid.union(locks.translate(INTF_BOARD_2D_OFFSET))
    
    support_w = 10
    support_h = 2
    support = (
        make_board_supporter(support_w, support_h, MAIN_BOARD_FRONT_GAP)
        .mirror("XY")
        .translate((MAIN_BOARD_3D_OFFSET[0], MAIN_BOARD_3D_OFFSET[1], 0))
    )
    solid = solid.union(support.translate((
        0, 
        MainBoard.H - 3 * MIL100 - support_h / 2, 
        0,
    )))
    
    support_w = 2
    support_h = 10
    support = (
        make_board_supporter(support_w, support_h,MAIN_BOARD_FRONT_GAP)
        .mirror("XY")
        .translate((MAIN_BOARD_3D_OFFSET[0], MAIN_BOARD_3D_OFFSET[1], 0))
    )
    solid = solid.union(support.translate((
        0, 
        MainBoard.H - support_h, 
        0,
    )))
    
    support_w = 2
    support_h = 10
    support = (
        make_board_supporter(support_w, support_h,MAIN_BOARD_FRONT_GAP)
        .mirror("XY")
        .translate((MAIN_BOARD_3D_OFFSET[0], MAIN_BOARD_3D_OFFSET[1], 0))
    )
    solid = solid.union(support.translate((
        0, 
        MainBoard.H - support_h, 
        0,
    )))
    
    support_w = 2
    support_h = 10
    support_tall = MAIN_BOARD_FRONT_GAP
    support_extra_tall = support_tall+ 2
    support_flange=3
    verts = [
        (0,0),
        (0,support_tall-1-support_w*3/2),
        (support_w,support_tall-1),
        (support_w,support_tall),
        (0,support_tall),
        (0,support_extra_tall),
        (-support_w,support_extra_tall),
        (-support_w,0),
    ]
    support = (
        cq.Workplane("XZ")
        .polyline(verts)
        .close()
        .extrude(-support_h)
    )
    support = support.union(
        cq.Workplane("XY")
        .box(support_flange, support_w, support_extra_tall, centered=False)
        .translate((-support_w-support_flange, 0, 0))
    )
    support = (
        support.rotate((0,0,0),(0,1,0),180)
        .translate(MAIN_BOARD_2D_OFFSET)
        .translate((MainBoard.W, 0, 0))
    )
    solid = solid.union(support)
    
    T_DC_JACK_WALL = T_PANEL
    dcjack_wall = (
        cq.Workplane("XY")
        .box(DC_JACK_MOUNT_W, T_DC_JACK_WALL, DC_JACK_MOUNT_H, centered=(True, False, False))
    )
    verts = [
        (0,0),
        (DC_JACK_MOUNT_H/2, 0),
        (0, DC_JACK_MOUNT_H * 2 / 3),
    ]
    wall_support = (
        cq.Workplane("YZ")
        .polyline(verts)
        .close()
        .extrude(T_DC_JACK_WALL)
        .translate((- DC_JACK_MOUNT_W/2, T_DC_JACK_WALL,0))
    )
    dcjack_wall = dcjack_wall.union(wall_support)
    dcjack_wall = dcjack_wall.union(wall_support.mirror("YZ"))
    dcjack_wall = (
        dcjack_wall.edges(">>Z and |Y")
        .chamfer(5)
    )
    dcjack_wall = dcjack_wall.cut(
        cq.Workplane("XY")
        .box(MJ_10A.SCREW_DIAMETER / 2, 99, 99, centered=(True, False, False))
        .translate((0, 0, DC_JACK_MOUNT_H/2))
    )
    solid = solid.union(
        dcjack_wall.translate((0,0, -DC_JACK_MOUNT_H/2))
        .mirror("XY").mirror("XZ")
        .translate(DC_JACK_HOLE_OFFSET)
    )
    solid = solid.cut(DC_JACK.hole_cutter)
    
    solid = (
        solid.faces(">Z")
        .edges("%circle")
        .chamfer(GENERIC_CHAMFER)
        )
    
    # 裏蓋固定用のフランジ
    flange = (
        cq.Workplane("XY")
        .box(PANEL_W, PANEL_H, H_FLANGE, centered=False)
    )
    flange = flange.cut(
        cq.Workplane("XY")
        .box(PANEL_W - T_FLANGE*2, PANEL_H -T_FLANGE*2, H_FLANGE, centered=False)
        .translate((T_FLANGE, T_FLANGE, 0))
    )
    solid = solid.union(flange.translate((0, 0, -H_FLANGE + T_PANEL)))
    
    solid = (
        solid
        .edges("((>>X or <<X) and (>>Y or <<Y)) or (>>Z and (>>X or <<X or >>Y or <<Y))")
        .chamfer(GENERIC_CHAMFER)
    )

    # 裏蓋固定用の爪穴
    cutter = (
        cq.Workplane("XY")
        .box(
            CASE_LOCK_DEPTH * 2 + GENERIC_GAP,
            CASE_LOCK_W + GENERIC_GAP,
            CASE_LOCK_H + GENERIC_GAP,
        )
    )
    for (pos, is_mirrored) in CASE_LOCK_POINTS:
        solid = solid.cut(cutter.translate(pos))

EXT_W = PANEL_W + T_PANEL * 2
EXT_H = PANEL_H + T_PANEL * 2
EXT_TALL = INNER_TALL + T_PANEL * 2

class BackCase:
    solid = (
        cq.Workplane("XY")
        .box(EXT_W, EXT_H, EXT_TALL, centered=False)
        .translate((-T_PANEL, -T_PANEL, -EXT_TALL + T_PANEL))
    )
    solid = solid.cut(
        cq.Workplane("XY")
        .box(PANEL_W, PANEL_H, INNER_TALL, centered=False)
        .translate((0, 0, -INNER_TALL))
    )
    solid = solid.cut(
        cq.Workplane("XY")
        .box(PANEL_W + NARROW_GAP * 2, PANEL_H + NARROW_GAP * 2, H_FLANGE + NARROW_GAP, centered=False)
        .translate((-NARROW_GAP, -NARROW_GAP, T_PANEL - H_FLANGE - NARROW_GAP))
    )
    
    # 基板固定
    holder_w = 3
    holder_shoulder_tall = INNER_TALL + MAIN_BOARD_3D_OFFSET[2]
    verts = [
        (2, 0),
        (-2, 0),
        (-2, holder_shoulder_tall + 1),
        (-1, holder_shoulder_tall + 2),
        (0, holder_shoulder_tall + 2),
        (0, holder_shoulder_tall - GENERIC_GAP),
        (2, holder_shoulder_tall - GENERIC_GAP),
    ]
    board_holder = (
        cq.Workplane("XZ")
        .polyline(verts)
        .close()
        .extrude(holder_w / 2, both=True)
        .translate((0, 0, -INNER_TALL))
    )
    board_holders = board_holder.translate((
        0,
        holder_w / 2,
        0,
    ))
    board_holders = board_holders.union(board_holder.translate((
        0,
        MainBoard.H - holder_w / 2,
        0
    )))
    board_holders = board_holders.union(board_holder.mirror("YZ").translate((
        MainBoard.W,
        12,
        0
    )))
    board_holders = board_holders.union(board_holder.mirror("YZ").translate((
        MainBoard.W,
        MainBoard.H - holder_w / 2,
        0
    )))
    solid = solid.union(board_holders.translate(MAIN_BOARD_2D_OFFSET))
    
    # ピンヘッダ用の穴
    solid = solid.cut(
        MainBoard.CN1.cutter
        .union(MainBoard.CN2.cutter)
        .edges("|Y")
        .fillet(GENERIC_CHAMFER)
        .translate(MAIN_BOARD_3D_OFFSET)
    )
    
    # DCジャックの穴
    solid = solid.cut(
        cq.Workplane("XY")
        .cylinder(20,MJ_10A.FRONT_DIAMETER/2, direct=(0,1,0))
        .translate(DC_JACK_HOLE_OFFSET)
    )
    
    # Picoのコネクタの穴
    solid = solid.cut(
        cq.Workplane("XY")
        .box(T_PANEL * 4, 12, 10, centered=(True, True, True))
        .edges("|X")
        .fillet(1)
        .translate(PICO_CONN_HOLE_OFFSET)
    )
    
    # 面取り
    solid = (
        solid.faces(">X or <X or >Y or <Y or >Z or <Z")
        .chamfer(GENERIC_CHAMFER)
    )
    
    # フロントパネル固定用の爪
    for (pos, is_mirrored) in CASE_LOCK_POINTS:
        notch = (
            cq.Workplane("XY")
            .box(
                CASE_LOCK_DEPTH * 2,
                CASE_LOCK_W,
                CASE_LOCK_H,
            )
            .edges(">>X and >>Z")
            .chamfer(CASE_LOCK_DEPTH + NARROW_GAP)
        )
        if is_mirrored:
            notch = notch.mirror("YZ")
        solid = solid.union(
            notch.translate(pos)
        )

front_panel = FrontPanel.solid
plate_out = FrontPanel.IND1.make_plate()
plate_err = FrontPanel.IND2.make_plate()
sw_key1 = FrontPanel.KEY1.solid
sw_key2 = FrontPanel.KEY2.solid
sw_key3 = FrontPanel.KEY3.solid

back_case = BackCase.solid


if False:
    cutter = (
        cq.Workplane("XY")
        .box(
            EXT_W + T_PANEL * 2,
            (INTF_BOARD_2D_OFFSET[1] + IntfBoard.SW1.y) * 2 - 4,
            99,
            centered=(False, True, True)
        )
        .translate((-T_PANEL * 2, 0, 0))
    )
    if True:
        front_panel = front_panel.cut(cutter)
        back_case = back_case.cut(cutter)
        sw_key1 = sw_key1.cut(cutter)
    else:
        front_panel = front_panel.intersect(cutter)
        back_case = back_case.intersect(cutter)
        sw_key1 = sw_key1.intersect(cutter)

plate_z_offset = T_PANEL - Indicator.T_PLATE
show_object(front_panel, options={"color": "#ccc"})

show_object(
    plate_out.translate(INTF_BOARD_2D_OFFSET)
    .translate((
        IntfBoard.LED1.x, IntfBoard.LED1.y, plate_z_offset,
    )),
    options={"color": "#08f"}
)
show_object(
    plate_err.translate(INTF_BOARD_2D_OFFSET)
    .translate((
        IntfBoard.LED2.x, IntfBoard.LED2.y, plate_z_offset,
    )),
    options={"color": "#f00"}
)

show_object(
    sw_key1.translate(INTF_BOARD_2D_OFFSET),
    options={"color": "#222"}
)
show_object(
    sw_key2.translate(INTF_BOARD_2D_OFFSET),
    options={"color": "#222"}
)
show_object(
    sw_key3.translate(INTF_BOARD_2D_OFFSET),
    options={"color": "#222"}
)


show_object(
    cq.Workplane("XY")
    .box(MainBoard.W, MainBoard.H, T_BOARD, centered=False)
    .translate(MAIN_BOARD_3D_OFFSET),
    options={"color": "#fc8"}
)

show_object(
    cq.Workplane("XY")
    .box(IntfBoard.W, IntfBoard.H, T_BOARD, centered=False)
    .translate(INTF_BOARD_3D_OFFSET),
    options={"color": "#fc8"}
)
show_object(
    cq.Workplane("XY")
    .box(SSD1906.W, SSD1906.H, SSD1906.T_BOARD, centered=False)
    .translate((IntfBoard.OLED.x, IntfBoard.OLED.y, T_BOARD))
    .translate(INTF_BOARD_3D_OFFSET),
    options={"color": "#048"}
)
show_object(
    cq.Workplane("XY")
    .box(SSD1906.Screen.W, SSD1906.Screen.H, SSD1906.Screen.T, centered=False)
    .translate((IntfBoard.OLED.x, IntfBoard.OLED.y, T_BOARD))
    .translate((SSD1906.Screen.X_OFFSET, SSD1906.Screen.Y_OFFSET, SSD1906.T_BOARD))
    .translate(INTF_BOARD_3D_OFFSET),
    options={"color": "#000"}
)
show_object(FrontPanel.VR1.solid,options={"color": "#ccc"})

show_object(RasPico.SOLID_TEMPLATE.rotate((0,0,0),(0,0,1),-90).translate((
    MAIN_BOARD_3D_OFFSET[0],
    MAIN_BOARD_3D_OFFSET[1] + 4.5 * MIL100,
    MAIN_BOARD_3D_OFFSET[2] + T_BOARD,   
)),options={"color": "#0f0"})

show_object(back_case, options={"color": "#111"})

front_panel = front_panel.rotate((0,0,0),(1,0,0),180)
front_panel.export("front_panel.step")
plate_out.export("plate_out.step")
plate_err.export("plate_err.step")
sw_key1.export("sw_key1.step")
sw_key2.export("sw_key2.step")
sw_key3.export("sw_key3.step")
back_case.export("back_case.step")


if False:
    test = front_panel.intersect(
        cq.Workplane("XY")
        .box(25, 25, 100, centered=True)
        .translate((12.5, 12.5, 0))
    )

    show_object(test)

    test.rotate((0,0,0),(1,0,0),180).export("tmp.test_panel.step")

