import numpy as np
import csv
from TimedData import TimedData

def csvLoadTransform(filename, timeCol, posCol, attCol, td, posID, attID):
    if( td.last == (-1) ):
        with open(filename, 'rb') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
            counter = 0
            for row in spamreader:
                try:
                    float(row[timeCol])
                    td.append();
                    td.d[td.last, td.timeID] = row[timeCol];
                    td.d[td.last, posID] = row[posCol:posCol+3]
                    td.d[td.last, attID] = row[attCol:attCol+4]
                    counter = counter + 1
                except ValueError:
                    print "Ignoring line " + str(counter)
        print("loading " + filename + " as transform, found " + str(counter) + " entries")
    else:
        print('Implement functionality when timedata is not empty');