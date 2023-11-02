from enum import Enum

Models_path = "/root/centerpose-ros2/models"

getModelPath = {
        0 : Models_path + "/bike_v1_140.pth",
        1 : Models_path + "/book_v1_140.pth",
        2 : Models_path + "/bottle_v1_sym_12_140.pth",
        3 : Models_path + "/camera_v1_140.pth",
        4 : Models_path + "/cereal_box_v1_140.pth",
        5 : Models_path + "/chair_v1_140.pth",
        6 : Models_path + "/cup_cup_v1_sym_12_140",
        7 : Models_path + "/cup_mug_v1_140.pth",
        8 : Models_path + "/laptop_v1_140.pth",
        9 : Models_path + "/shoe_v1_140.pth"
    }

class CenterPoseModels(Enum):
    BIKE = 0
    BOOK = 1
    BOTTLE = 2
    CAMERA = 3
    CEREAL = 4
    CHAIR = 5
    CUP = 6
    MUG = 7
    LAPTOP = 8
    SHOE = 9

    getModelPath = {
        0 : Models_path + "/bike_v1_140.pth",
        1 : Models_path + "/book_v1_140.pth",
        2 : Models_path + "/bottle_v1_sym_12_140.pth",
        3 : Models_path + "/camera_v1_140.pth",
        4 : Models_path + "/cereal_box_v1_140.pth",
        5 : Models_path + "/chair_v1_140.pth",
        6 : Models_path + "/cup_cup_v1_sym_12_140",
        7 : Models_path + "/cup_mug_v1_140.pth",
        8 : Models_path + "/laptop_v1_140.pth",
        9 : Models_path + "/shoe_v1_140.pth"
    }

    def get_path(self):
        return CenterPoseModels.getModelPath[int(self.value)]

class CenterPoseTrackModels(Enum):
    MUG = Models_path + "/cup_mug_15.pth"

class archType(Enum):
    NOTRACKING = 'dlav1_34'
    TRACKING = 'dla_34'

class experiment_type(Enum):
    LIVE = 1
    VIDEO = 2

if __name__ == '__main__':
    my_model = CenterPoseModels.MUG
    print(my_model.name)
    print(my_model.value)
    print(my_model.get_path())