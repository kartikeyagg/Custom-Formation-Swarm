# import csv
# file = open('~/Kartikey_swarm/traj1.csv')
# csvreader = csv.reader(file)
import csv
file = open('/home/kartikey/Kartikey_swarm/traj1.csv')
csvreader = csv.reader(file)
header = next(csvreader)
print(header)
rows = []
for row in csvreader:
    rows.append(row)
for i in range(len(rows)):
    for j in range(len(rows[i])):
        rows[i][j] = float(rows[i][j])

print(rows)
file.close()
