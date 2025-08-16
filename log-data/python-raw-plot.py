import matplotlib.pyplot as plt
import csv

rec_num = []
ax = []
ay = []
x = 0
x_axes = []

with open('raw-log.csv') as csvfile:
	plots = csv.reader(csvfile, delimiter=',')

	for row in plots:

		if len(row) != 0:
			# print(float(row[3]))
			# print(row)
			x += 1
			x_axes.append(x)
			# rec_num.append(row[0])
			ax.append(row[4])
			ay.append(row[5])

y = 0
with open('x-extracted.csv', 'w', newline='') as f:
	writer = csv.writer(f)
	for val in ax:
		writer.writerow([y, val])
		y += 1


# plt.plot(x_axes, ax, color='r', label='x acceleration')
# plt.plot(x_axes, ay, color='g', label='y acceleration')
# plt.legend()
# plt.show()
