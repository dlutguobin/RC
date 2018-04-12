# RC
RC based on Moveit
# 无需列表
- 文本
- 文本
- 文本
## 有序列表
1. 文本
2. 文本
3. 文本
### 引用
> Although MoveIt! is a widely used motion planning framework and ROS package, its documentation needs a bit of updating. The website and tutorials consistently get minor improvements and contributions from the dedicated community, but a much larger holistic effort is needed in improving the overall information flow and ensuring that all major concepts in MoveIt! are included.
#### 粗体和斜体
*斜体* **粗体** 正常
##### 代码引用
`typedef boost::function<void(const MConstPtr&, FilterFailureReason)> tf::MessageFilter< M >::FailureCallback`
```
message_filters::Subscriber<MessageType> sub(node_handle_, "topic", 10);
tf::MessageFilter<MessageType> tf_filter(sub, tf_listener_, "/map", 10);
tf_filter.registerCallback(&MyClass::myCallback, this);
```
###### 链接和图片

[主页](https://github.com/dlutguobin)

![](http://moveit.ros.org/assets/images/Franka_Emika_Panda.jpg)
