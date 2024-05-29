import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# 读取Excel文件
df = pd.read_excel('simulation_results-modified.xlsx')

# 设置绘图风格
sns.set(style="whitegrid")

metrics = ['Mean Duration', 'Collisions', 'CO', 'CO2', 'HC', 'PMx', 'NOx', 'Fuel Consumption']
titles = ['Mean Duration by Model and Flow', 'Collisions by Model and Flow',
          'CO by Model and Flow', 'CO2 by Model and Flow', 'HC by Model and Flow',
          'PMx by Model and Flow', 'NOx by Model and Flow', 'Fuel Consumption by Model and Flow']

for metric, title in zip(metrics, titles):
    plt.figure(figsize=(10, 6))
    lineplot = sns.lineplot(data=df, x='Flow', y=metric, hue='Model', style='Model', markers=True, dashes=False)
    plt.title(title)
    plt.xticks(rotation=45)  # 旋转x轴标签以显示完全
    for line in lineplot.lines:
        for x_value, y_value in zip(line.get_xdata(), line.get_ydata()):
            plt.text(x_value, y_value, f'{y_value:.2f}', color=line.get_color())
    plt.tight_layout()
    plt.savefig(f'figures/{title}.png')
    plt.close()
