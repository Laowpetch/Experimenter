from utils import utils

if __name__ == '__main__':
    utils_ = utils()
    rMat,tvec = utils_.calibration()
    print(rMat)
    print(tvec)