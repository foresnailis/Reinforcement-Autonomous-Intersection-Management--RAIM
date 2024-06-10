# Reinforcement Autonomous Intersection Management - RAIM
This is the repository of the code used for **RAIM**, a course project

## Installation

### SUMO install
```bash
bash install_sumo.sh
```

### python env
```bash
conda create -n RAIM python=3.8
conda activate RAIM
pip install --f requirements.txt
```

## How to run
Just run the main.py file
```bash
# TD3 默认开启课程学习、策略噪声, 关闭跟驰与变道模型
python main.py --model_name=TD3-CL

# TD3 关闭课程学习、策略噪声, 关闭跟驰与变道模型, 限速15
python main.py --model_name=TD3 --class_learn=False --policy_noise=False --maxSpeed=15

# TD3 关闭课程学习、策略噪声, 开启跟驰与变道模型
python main.py --model_name=TD3-CF --class_learn=False --policy_noise=False --cf=True

# DDPG 默认开启课程学习, 关闭跟驰与变道模型
python main.py --model_name=DDPG-CL --agent=DDPG
```

## How to test
Prepare pth file in /ckpt/TD3 or others before excuting the test.
```
python main_test.py
```