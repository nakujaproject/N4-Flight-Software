# generate dummy csv file using a linear regression model

import csv
import numpy as np

years = np.random.randint(1,10 +1, 1000)
salaries = years * 200 + 2000 + np.random.randint(1, 400 + 1, 1000)

with open('dummy.csv', 'w') as file:
    writer = csv.writer(file)
    writer.writerow(('years', 'salary'))
    writer.writerows(zip(years, salaries))