import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

# prepare sample data
items  = ['Item 1', 'Item 2', 'Item 3', 'Item 4', 'Item 5']
sales_data = dict(zip(items, np.random.randint(0, 25, (5, 30))))
item_sales = pd.DataFrame(sales_data)

fig, ax = plt.subplots(figsize=(8,4))

sns.set_palette("tab10", n_colors=5)
sns.lineplot(x=item_sales.index, y='Item 1', data=item_sales, alpha=0.3, ax=ax)
sns.lineplot(x=item_sales.index, y='Item 2', data=item_sales, alpha=0.3, ax=ax)
sns.lineplot(x=item_sales.index, y='Item 3', data=item_sales, alpha=0.3, ax=ax)
sns.lineplot(x=item_sales.index, y='Item 4', data=item_sales, alpha=1, ax=ax)
sns.lineplot(x=item_sales.index, y='Item 5', data=item_sales, alpha=0.3, ax=ax)
ax.set_ylabel('')
ax.set_yticks([])
plt.title('Timeline of item sales')
plt.show()