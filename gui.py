import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
class App:
    def __init__(self):
        # 初始化实例
       	gui.Application.instance.initialize()	
        
        # 创建主窗口
        self.window = gui.Application.instance.create_window('My First Window', 800, 600)
        
        # 创建显示场景
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        
        # 将场景添加到窗口中
        self.window.add_child(self.scene)
        
        # 创建一个球
        sphere = o3d.geometry.TriangleMesh.create_sphere()
        sphere.paint_uniform_color([0.0, 1.0, 1.0])
        sphere.compute_vertex_normals()
        material = rendering.Material()
        material.shader = 'defaultLit'
        
        # 将球加入场景中渲染
        self.scene.scene.add_geometry("Sphere", sphere, material)
        
        # 设置相机属性
        bounds = sphere.get_axis_aligned_bounding_box()
        self.scene.setup_camera(60, bounds, bounds.get_center())
	
    def run(self):
        gui.Application.instance.run()

if __name__ == "__main__":
    app = App()
    app.run()
