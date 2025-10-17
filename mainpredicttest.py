from DLpredict.predict import UnetPackage

def main():
    # 创建UnetPackage实例，并设置所需的参数
    unet_package = UnetPackage(
        mode='video',  # 设置模式为video
        # 设置视频路径或摄像头索引，视频源或者0，1，2... 
        #1为笔记本相机，2为深度相机
        video_path= 0,  
        # video_path='assets\demo6.18-480.mp4',
        video_save_path='',  # 设置视频保存路径
        video_fps=30  # 设置视频帧率
    )
    
    # 调用视频处理方法
    unet_package.video()

if __name__ == "__main__":
    main()