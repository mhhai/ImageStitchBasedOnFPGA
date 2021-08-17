# ImageStitchBasedOnFPGA
毕业了，说下七路摄像头拼接。先说下做的不好的地方：<br>
1.图像采集模块没有做的怎么好，白平衡之类的没做好，这也是师弟一直在做；<br>
2.图像融合部分的小数权重没有怎么做好，后面补上。<br>
再说一下，项目实现需要注意的地方：<br>
1.采集的像素数据是24bit，DDR3控制器的接口是64bit，也就是说需要自己写一个24bit到64bit的异步FIFO转换模块。这部分网上资料很少，自己实现需要注意以下几点：<br>
a.存储器是用Block RAM还是二维寄存器数组。Block RAM不能用，因为输入输出的位宽需要满足2^n次方关系，因此好的办法是用二维寄存器数组了。用二维寄存器数组，在写入像素数据时时钟频率不能太高，否则写入的数据会出错。实测写端200MHz出错，100MHz可以。<br>
b.输入是24bit，输出是64bit，该异步FIFO的width应该是8bit，depth根据一次突发传输的长度可以自己定，depth太高的话，编译时间会大大加长，而且depth肯定是2^n次方；<br>
c.读指针一次加3，写指针一次加8，因此转换成格雷码来降低亚稳态出现的概率已经不行了，需要根据写端逻辑来控制读端逻辑：只要异步FIFO中的数据满足一次DDR3突发传输的长度，那么就将数据写入到DDR3中。<br>
2.圆柱面投影算法实现<br>
a.至少OpenCV中怎么实现图像拼接的上万行代码要搞懂；<br>
b.圆柱面投影中是用双线性插值算法还是最近邻插值算法：<br>
双线性插值算法：需要在一个时钟周期取出四个待插值点的四个像素，那么有两种解决方法，一是将从DDR3中读取出的数据存到四个相同的Block RAM中，然后产生不同的地址坐标从Block RAM中读取这四个像素点，这种方法的缺点是片上资源占用率很大，好处简单。第二种方法将像素按照奇偶行奇偶列存放到不同的Block RAM中，这部分我仿真是没有问题的，但是下板的时序不对，因此我还是用的第一种方法。<br>
最近邻插值算法：最近邻插值算法比较简单实用，很容易实现，工程应用比较推荐。<br>
3.圆柱面投影的时序问题：<br>
这部分自我感觉实现得还可以，还想看能不能发篇论文，因此有需要的可以交流交流，具体就不多说了。<br>
总体来说，七路摄像头拼接还是不容易的，即便现在还要好多路要走。<br>
