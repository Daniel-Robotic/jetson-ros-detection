import os
import cv2
import numpy as np


def search_chessboard_pattern(img, size=(6,9), conv_size=(3,3)):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) 
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
    ret, corners = cv2.findChessboardCorners(gray,
                                            size,
                                            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        corners = cv2.cornerSubPix(gray, corners,
                                    conv_size, (-1, -1), criteria)
        
        return ret, corners
    
    return False, None


def mono_calibration(path, 
                     size=(6,9), 
                     images_format_valid=['png'], 
                     conv_size=(3,3)):
    
    try:
        
        assert size[0] > 0, f"Первое значение {size[0]} размера шахматной доски <= 0"
        assert size[1] > 0, f"Второе значение {size[1]} размера шахматной доски <= 0"
        assert conv_size[0] > 0, f"Первое значение {conv_size[0]} размера свертки <= 0"
        assert conv_size[1] > 0, f"Второе значение {conv_size[1]} размера свертки <= 0"       
        assert conv_size[0] == conv_size[1], f"Размеры свертки не одинаковы: {conv_size}"
        assert conv_size[0] % 2, "Размер свертки должен быть не четным и не равен 0"
        
        
        objp = np.zeros((size[0] * size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2)
        
        imgpoints = []
        objpoints = []
        img_correct = 0
        images_path = list(filter(lambda file: file.split(".")[-1] in images_format_valid, os.listdir(path)))
            
        
        for fname in images_path:
            img_original = cv2.imread(f'{path}/{fname}')
            
            ret, corners = search_chessboard_pattern(img=img_original,
                                                     size=size,
                                                     conv_size=conv_size)
            
            if ret:
                objpoints.append(objp)
                imgpoints.append(corners)
                img_correct += 1
                
        
        height, width = img_original.shape[:2]
        
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints,
                                                           imgpoints,
                                                           (width, height),
                                                           None, None)
            
        msg = f'Удалось обработать {img_correct} изображений из {len(images_path)}'
        
    except AssertionError as e:
        ret, mtx, dist = False, None, None
        msg = f'Ошибка каллибровки: {e}'
        
    except Exception as e:
        ret, mtx, dist = False, None, None
        msg = f'Ошибка каллибровки: {e}'
    
    finally:
        return ret, mtx, dist, msg
    
