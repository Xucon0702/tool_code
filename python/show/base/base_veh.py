#宏定义

from ctypes import Structure, c_float


veh_type = "A58"

if(veh_type == "A58"):
    print("veh is A58")
    #GAC_A58车车身参数,单位/m
    AXIS_DISTANCE  = 2.730
    VEHICLE_LEN	= 4.650
    VEHICLE_WID	= 1.887
    FRONT_EDGE2CENTER = 3.674
    REAR_EDGE2CENTER  = 0.976
    SIDE_EDGE2CENTER  = VEHICLE_WID / 2
    WHEEL_BASE        = AXIS_DISTANCE
else:
    print("unknow veh type")


class location(Structure):  #结构体的使用
    _fields_ = [
        ('x',c_float),
        ('y',c_float),
        ('yaw',c_float),
    ]
    _pack_ = 1              #字节对齐

class C_base_veh:
    def __init__(self,x,y,yaw):     
        self.x=x
        self.y=y
        self.yaw=yaw

    def CalCornerCoordinate():
        print("AXIS_DISTANCE: ",AXIS_DISTANCE)
        # a = location(0,0,0)
        # print("a: ",a)
        return

    # def CalCornerCoordinate( LocationPoint tVehicleRearAxleCenter,
    #                       LocationPoint& tRightRearCornerPoint,
    #                       LocationPoint& tRightFrontCornerPoint,
    #                       LocationPoint& tLeftFrontCornerPoint,
    #                       LocationPoint& tLeftRearCornerPoint):
    #     tRightRearCornerPointOpposite  = C_base_veh(0,0,0)
    #     tRightFrontCornerPointOpposite = C_base_veh(0,0,0)
    #     tLeftFrontCornerPointOpposite  = C_base_veh(0,0,0)
    #     tLeftRearCornerPointOpposite   = C_base_veh(0,0,0)
    #     tOriginPoint                   = C_base_veh(0,0,0)
    #     # tOriginPoint.x=0
    #     # tOriginPoint.y=0
    #     tRightRearCornerPointOpposite.x = -REAR_EDGE2CENTER
    #     tRightRearCornerPointOpposite.y = -VEHICLE_WID / 2

    #     tRightFrontCornerPointOpposite.x = VEHICLE_LEN - REAR_EDGE2CENTER
    #     tRightFrontCornerPointOpposite.y = tRightRearCornerPointOpposite.y 

    #     tLeftFrontCornerPointOpposite.x = tRightFrontCornerPointOpposite.x 
    #     tLeftFrontCornerPointOpposite.y = VEHICLE_WID / 2

    #     tLeftRearCornerPointOpposite.x = tRightRearCornerPointOpposite.x
    #     tLeftRearCornerPointOpposite.y =tLeftFrontCornerPointOpposite.y

    #     RotateCoordinateOfPoint(tOriginPoint, tRightRearCornerPointOpposite, tVehicleRearAxleCenter.yaw, tRightRearCornerPoint)
    #     tRightRearCornerPoint.x = tVehicleRearAxleCenter.x + tRightRearCornerPoint.x 
    #     tRightRearCornerPoint.y = tVehicleRearAxleCenter.y + tRightRearCornerPoint.y

    #     RotateCoordinateOfPoint(tOriginPoint,tRightFrontCornerPointOpposite, tVehicleRearAxleCenter.yaw, tRightFrontCornerPoint)
    #     tRightFrontCornerPoint.x= tVehicleRearAxleCenter.x + tRightFrontCornerPoint.x
    #     tRightFrontCornerPoint.y = tVehicleRearAxleCenter.y + tRightFrontCornerPoint.y

    #     RotateCoordinateOfPoint(tOriginPoint,tLeftFrontCornerPointOpposite, tVehicleRearAxleCenter.yaw, tLeftFrontCornerPoint)
    #     tLeftFrontCornerPoint.x = tVehicleRearAxleCenter.x + tLeftFrontCornerPoint.x
    #     tLeftFrontCornerPoint.y = tVehicleRearAxleCenter.y + tLeftFrontCornerPoint.y

    #     RotateCoordinateOfPoint(tOriginPoint, tLeftRearCornerPointOpposite, tVehicleRearAxleCenter.yaw, tLeftRearCornerPoint)
    #     tLeftRearCornerPoint.x = tVehicleRearAxleCenter.x + tLeftRearCornerPoint.x
    #     tLeftRearCornerPoint.y = tVehicleRearAxleCenter.y + tLeftRearCornerPoint.y


    


#ifdef A58

#endif