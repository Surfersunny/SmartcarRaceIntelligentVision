**基于NXP-eIQ官方机器学习平台的视觉模型训练**

Reference--全网较完整入门教程演示 <https://blog.csdn.net/2403_87969572/article/details/153585855>

# 1.下载eIQ Portal
- 网址：https://www.nxp.com.cn/design/design-center/software/eiq-ai-development-environment/eiq-toolkit-for-end-to-end-model-development-and-deployment:EIQ-TOOLKIT
- 选择 eIQ Toolkit Windows下载

# 2.创建Structured Folder并选择Import Dataset导入数据集
- 最顶层命名为dataset 次顶层train test 第三层各图片类型名文件夹 其内放入所采集的每一类图片 **注意这个文件夹格式必须严格对齐！**
- **力推使用该魔改上位机快速采集数据集！** 鼠标/空格切换同一类型不同图片 Openart定时拍照存入sd卡然后剪切粘贴到电脑文件夹即可 <https://github.com/Hey9990/Smartcar-VR-DatasetGenerator>
- 导入数据集并创建project即可 eIQ会根据结构文件夹自动分好图片

# 3.模型类型选择（通常就是mobilenet_v2）+参数设置+训练+测试集验证+导出tflite（轻量化嵌入式部署神经网络）
- 细节仍参见上述Reference文档

总而言之，对于NXP系列单片机的机器学习模型训练与部署，强推或者说**必须使用eIQ**！其性能+兼容性总体足够令人满意，自行编写Pytorch/Tensorflow代码可以在自己上位机跑着玩玩但转为tflite格式嵌入式部署非常困难，建议直接选eIQ（而且0代码量，GUI界面鼠标点点就完事）！