## 5号步兵 Draft1 --Instruction

### 代码框架：

![image-20221214202547015](C:\Users\Max\Desktop\image-20221214202547015.png)

### C板代码 --- Instruction

#### Tasks

+ communication : 实现板间通讯

  + 电机电压赋0 防炸机+
  + 当收到信号后为can_flag赋值为1 防炸机+

+ weopen : 摩擦轮控制

  摩擦轮电机id号：

  + pid_init   P:       I:      D:
  + 遥控器s[0]调至挡位2 开火 3508 ~~全速旋转~~ 

```
if (rc_ctrl.rc.s[0] == 2)
    {
      target_speed[0] = 300;
      target_speed[1] = 300;
      target_speed[2] = 300;
      set_motor_voltage(0, motor_info[0].set_voltage, motor_info[1].set_voltage, motor_info[2].set_voltage,0);
    }
```

+ gimbal_pitch : 云台pitch轴控制

  6020id号：5

  + pid_init   P:       I:      D:

#### Variables

#### Struct

### A板代码 --- Instruction

