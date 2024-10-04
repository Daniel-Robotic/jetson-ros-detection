import os
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from std_srvs.srv import Empty
from sensor_msgs.msg import CompressedImage


from calibration_interfaces.msg import CalibrationData
from calibration_interfaces.srv import (ChangeState, 
                                        CompressedImageService,
                                        CameraCalibration,
                                        CalibrationInfo)

from ament_index_python.packages import get_package_share_directory

from calibration.utils import search_chessboard_pattern, mono_calibration

class CalibrationNode(Node):
    # TODO Добавь папку Launch, через которуюю будет производиться запуск модуля калибровки
    # А также модуля распознования ключевых точек оператора в рабочей зоне коллаборавтиного робота
    def __init__(self):
        super().__init__('calibration_node_server')
        
        package_share_directory = get_package_share_directory("calibration")
        
        # Внутренние переменные системы
        self.camera_mode = False
        self.cv_bridge = CvBridge()
        self.images_format_valid = ["png", "jpeg", "jpg"]
        
        # Получение параметров от пользователя
        self.declare_parameter('image_folder', os.path.join(package_share_directory, 'calibration_images'))
        
        # Сервисы ROS2
        self.camera_mode_service_ = self.create_service(ChangeState,
                                                        "toggle_camera_mode",
                                                        self.toggle_camera_mode_callback)
        
        self.get_frame_service_ = self.create_service(CompressedImageService,
                                                      "get_calibration_frames",
                                                      self.get_calibration_frames_callback)
        
        self.mono_calibration_service_ = self.create_service(CameraCalibration,
                                                             'camera_mono_calibration',
                                                             self.mono_calibration_callback)
        
        self.get_mono_calibration_info_service_ = self.create_service(CalibrationInfo,
                                                                        "get_mono_calibration_info",
                                                                        self.get_mono_calibration_info_callback)
        
        self.clear_calibration_info_service_ = self.create_service(Empty,
                                                                    "clear_calibration_info",
                                                                    self.clear_calibration_info_callback)
        
        self.clear_calibration_images_service = self.create_service(Empty,
                                                                    'clear_calibration_images',
                                                                    self.clear_calibration_images_callback)
        
        # Топики ROS2
        self.frame_publisher_ = self.create_publisher(CompressedImage,
                                                      "camera/frame",
                                                      1)
        
        # TODO Вернуть на место после добавления камеры
        # Таймеры работы для топиков
        # self.timer_ = self.create_timer(1.0/30, self.send_frame)
        
        
        self.get_logger().info('Calibration module has been started...')
    
    def get_calibration_frames_callback(self, 
                           request: CompressedImageService.Request,
                           response: CompressedImageService.Response) -> CompressedImageService.Response:
        """
        Сервис позволяющий получить все изображения для калибровки текущей камеры
        или при калибровке мультикамерной системы
        
        Args:
            request (CompressedImageService.Request): Пустой запрос на сервер

        Returns:
            CompressedImageService.Response: Возвращает три переменные:
                `images` (sensor_msgs/CompressedImage) - Массив изображений
                `status` (status) - статус операции: `true` - изображения удалось получить и передать, `false` - изображения не получены
                `msg` (string) - отладочная информация
        """       
        imgs_folder = self.get_parameter('image_folder').get_parameter_value().string_value
        
        try:
            for img in list(filter(lambda file: file.split(".")[-1] in self.images_format_valid, os.listdir(imgs_folder))):
                im = cv2.imread(os.path.join(imgs_folder, img))
                im = cv2.resize(im, (640, 480)) # TODO Убрать данный функционад
                
                response.images.append(self.cv_bridge.cv2_to_compressed_imgmsg(im))
            
            if not response.images:
                raise FileNotFoundError(f"Не удалось получить изображения в папке {imgs_folder}")
            
            response.status = True
            response.msg = f"Было получено: {len(response.images)} изображений"
        
        except FileNotFoundError:
            response.status = False
            response.msg = f"""Не удалось найти файлы по пути: {imgs_folder}. 
Попробуйте указать полный путь до дериктории или поместить в нее калибровочные изображения, с поддерживаемым форматом файлов: {self.images_format_valid}"""
            self.get_logger().error(response.msg)
            
        finally:
            return response

    # Services
    def template_detected_info_callback(self):
        # TODO метод который будет принимать пустое сообщение
        # Если удалось распознать шаблон -> создаем или сразу сохраняем в папке
        # Возвращаем status: true
        
        pass
    
    
    def mono_calibration_callback(self,
                                  request: CameraCalibration.Request,
                                  response: CameraCalibration.Response) -> CameraCalibration.Response:
        """
        Сервис выполняющий калибровку камеры

        Args:
            request (CameraCalibration.Request): Принимает две переменные:
                `chessboard_size` (int16) - массив из двух элементов, характерезующий размер шахматной доски
                'convolution_size' (int8) - размер свертки
        Returns:
            CameraCalibration.Response: данные о калибровки (`calibration_info`) представленные сообщением `CalibrationData` содержащее:
                `status` (bool) - статус операции: `true` - калибровка прошла успешно, `false` - ошибка калибровки
                `msg` (str) - отладочная информация
                `camera_matrix` (float64) - массив из 9 элементов описывающий матрицу камеры
                `distortion_coefficients` (float64) - массив из 5 элементов описывающий дисторисю камеры
        """
        imgs_folder = self.get_parameter('image_folder').get_parameter_value().string_value
        
        ret, mtx, dist, msg = mono_calibration(path=imgs_folder,
                                          size=tuple(request.chessboard_size),
                                          images_format_valid=self.images_format_valid,
                                          conv_size=(request.convolution_size, request.convolution_size))
        
        if not ret:
            response.calibration_info = CalibrationData(status=False,
                                                        msg=msg,
                                                        camera_matrix=[0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                        distortion_coefficients=[0, 0, 0, 0, 0])
            self.get_logger().error(response.calibration_info.msg)
            return response
        
        response.calibration_info = CalibrationData(status=True,
                                                    msg=msg,
                                                    camera_matrix=mtx.flatten(),
                                                    distortion_coefficients=dist.flatten())
        self.get_logger().info(response.calibration_info.msg)
        
        np.savez(os.path.join(get_package_share_directory("calibration"), "calibration_info.npz"), 
                 mtx=mtx, dist=dist)
                
        return response
    
    def get_mono_calibration_info_callback(self,
                                           request: CalibrationInfo.Request,
                                            response: CalibrationInfo.Response) -> CalibrationInfo.Response:
        
        """
        Сервис получения информации об каллибровки камеры

        CameraCalibration.Response: данные о калибровки (`calibration_info`) представленные сообщением `CalibrationData` содержащее:
                `status` (bool) - статус операции: `true` - калибровка прошла успешно, `false` - ошибка калибровки
                `msg` (str) - отладочная информация
                `camera_matrix` (float64) - массив из 9 элементов описывающий матрицу камеры
                `distortion_coefficients` (float64) - массив из 5 элементов описывающий дисторисю камеры
        """
        
        try:
            data = np.load(os.path.join(get_package_share_directory("calibration"), "calibration_info.npz"))
            response.calibration_info = CalibrationData(status=True,
                                                    msg=f"Калибровчные данные, были получены успешно",
                                                    camera_matrix=data['mtx'].flatten(),
                                                    distortion_coefficients=data['dist'].flatten())
            self.get_logger().info(response.calibration_info.msg)
            
        except FileNotFoundError:
            response.calibration_info = CalibrationData(status=False,
                                                        msg=f"Не удалось калибровочные данные. Перекалибруйте камеру занова",
                                                        camera_matrix=[0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                        distortion_coefficients=[0, 0, 0, 0, 0])
            self.get_logger().warn(response.calibration_info.msg)
            
        finally:
            return response
        
    def clear_calibration_info_callback(self,
                                        request: Empty.Request,
                                        response: Empty.Response) -> Empty.Response:
        
        """
        Сервис по очистке данных о калибровки
        """
                
        try:
            os.remove(os.path.join(get_package_share_directory("calibration"), "calibration_info.npz"))
            self.get_logger().info('Калибровочные данные успешно очищены')
        except FileNotFoundError:
            self.get_logger().error('Калибровочные данные отсутсвуют, невозможно произвести отчистку')
        finally:
            return response
    
    def clear_calibration_images_callback(self,
                                          request: Empty.Request,
                                          response: Empty.Response) -> Empty.Response:
        
        """
        Сервис по очистке калибровочных изображений с папки
        """
        
        imgs_folder = self.get_parameter('image_folder').get_parameter_value().string_value
        
        if not os.path.exists(imgs_folder):
            self.get_logger().warn(f'Папка {imgs_folder} не существует')
            return response
        
        for imgname in os.listdir(imgs_folder):
            img_path = os.path.join(imgs_folder, imgname)
            
            if os.path.isfile(img_path):
                os.remove(img_path)
        
        self.get_logger().info(f'Все калибровочные изображения с папки: {imgs_folder} - успешно удалены')
        
        return response
        
    
    def toggle_camera_mode_callback(self,
                                    request: ChangeState.Request,
                                    response: ChangeState.Response) -> ChangeState.Response:
        """
        Изминение режима работы камеры (включена/выключена)

        Args:
            request (ChangeState.Request): Запросом сервиса служит `bool` 
                значение переменной `state`. `state` указывает на то, 
                какое значение необходимо установить на данный момент 
                
        Returns:
            ChangeState.Response: Ответом сервиса служит две переменные:
                1) `status` (bool) - Статус изминения объекта `true` - состояние успешно изменено, `false` - состояние не возможно было изменить
                2) `msg` (string) - Информация об изминении состояния
        """
        
        if request.state == self.camera_mode:
            response.msg = f"[!] Невозможно изменить состояние камеры: {self.get_camera_state()} на {self.get_camera_state()}"
            response.status = False
            self.get_logger().warn(response.msg)
            
            return response
        
        # TODO Включение камеры + получение ошибки во время включения + остановка всех сервисов и топикв
        
        response.status = True
        self.camera_mode = request.state
        response.msg = f"Камера переключена в режим: {self.get_camera_state()}"
        self.get_logger().info(response.msg)
        
        return response
    
    # MSG
    def send_frame(self) -> None:
        """
            Метод отправки для отправки кадра с камеры в прямом потоке
        """
        
        # TODO Заменить на код получения кадра с камеры
        img = cv2.imread(f"{self.get_parameter('image_folder').get_parameter_value().string_value}/Pattern.png")
        # TODO Параметры должен получать из вне или конфиг файл
        ret, corners = search_chessboard_pattern(img, (9, 6))
        
        if ret:
            img = cv2.drawChessboardCorners(img, (9, 6),
                                            corners, ret)
        
            msg = self.cv_bridge.cv2_to_compressed_imgmsg(img)
            self.frame_publisher_.publish(msg)
        
        else:
            msg = self.cv_bridge.cv2_to_compressed_imgmsg(img)
            self.frame_publisher_.publish(msg)
        
    # Вспомогательные методы
    def get_camera_state(self) -> str:
        """
        Получение состояние камеры (включена/выключена)
        за счёт статуса переменной self.camera_mode

        Returns:
            str: Состояние камеры (ВКЛЮЧЕНО/ВЫКЛЮЧЕНО)
        """
        return 'включено'.upper() if self.camera_mode else 'выключено'.upper()



def main(args=None):
    rclpy.init(args=args)
    server = CalibrationNode()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
