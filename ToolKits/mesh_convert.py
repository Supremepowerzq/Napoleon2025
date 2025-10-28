import vtk
import os

def convert_vtu2vtk(input_path):
    

def convert_to_vtk(input_path):
    # 获取文件扩展名
    _, file_extension = os.path.splitext(input_path)
    
    # 根据文件类型初始化相应的读取器
    if file_extension.lower() == '.vtp':
        reader = vtk.vtkXMLPolyDataReader()
    elif file_extension.lower() == '.vtu':
        reader = vtk.vtkXMLUnstructuredGridReader()
    else:
        print("Unsupported file format:", file_extension)
        return
    
    # 设置输入文件名并更新读取器
    reader.SetFileName(input_path)
    reader.Update()
    
    # 准备输出文件名（更改扩展名为.vtk）
    output_path = os.path.splitext(input_path)[0] + ".vtk"
    
    # 根据输入文件类型初始化相应的写入器，并设置输出文件名
    if file_extension.lower() == '.vtp':
        writer = vtk.vtkPolyDataWriter()
        writer.SetInputData(reader.GetOutput())
    elif file_extension.lower() == '.vtu':
        writer = vtk.vtkUnstructuredGridWriter()
        writer.SetInputData(reader.GetOutput())
    
    writer.SetFileName(output_path)
    writer.Write()
    print(f"Converted {input_path} to {output_path}")

def visualize_model(input_path):
    # 获取文件扩展名
    _, file_extension = os.path.splitext(input_path)
    
    # 根据文件类型初始化相应的读取器
    if file_extension.lower() == '.vtp':
        reader = vtk.vtkXMLPolyDataReader()
    elif file_extension.lower() == '.vtu':
        reader = vtk.vtkXMLUnstructuredGridReader()
    else:
        print("Unsupported file format:", file_extension)
        return
    
    reader.SetFileName(input_path)
    reader.Update()

    # 创建映射器
    if file_extension.lower() == '.vtp':
        mapper = vtk.vtkPolyDataMapper()
    elif file_extension.lower() == '.vtu':
        mapper = vtk.vtkDataSetMapper()
    
    mapper.SetInputConnection(reader.GetOutputPort())

    # 创建演员
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    # 创建渲染器、渲染窗口和渲染窗口交互器
    renderer = vtk.vtkRenderer()
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    # 添加演员到渲染器
    renderer.AddActor(actor)
    renderer.SetBackground(0.1, 0.2, 0.4)  # 设置背景颜色

    # 开始交互
    renderWindow.Render()
    renderWindowInteractor.Start()

if __name__ == "__main__":
    # 示例：转换文件
    input_path = "Dataset\\MedModels\\0066_H_CORO_H\\Meshes\\0002_0001.vtu"  # 更改为您的文件路径
    visualize_model(input_path)
