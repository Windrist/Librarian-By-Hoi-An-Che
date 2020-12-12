import csv
import sys
import os
import csv

def checkBook(book):
    file = os.path.dirname(__file__)
    csv_file = csv.reader(open(file + '/../data/Borrow.csv', 'r'), delimiter=",")
    for row in csv_file:
        if book == row[0]:
            return row[1], int(row[2]), int(row[3]), int(row[4])
    return "", -1, -1, -1

def checkName(name, book):
    file = os.path.dirname(__file__)
    csv_file = csv.reader(open(file + '/../data/Return.csv', 'r'), delimiter=",")
    book_list = []
    is_book = False
    is_another_book = False
    author = ""
    for row in csv_file:
        if name == row[0]:
            if book == row[1]:
                is_book = True
                author = row[2]
            else:
                is_another_book = True
                book_list.append(row[1])
    if is_book == True:
        if is_another_book == True:
            return True, True, book_list, author
        else:
            return True, False, book_list, author
    else:
        if is_another_book == True:
            return False, True, book_list, author
        else:
            return False, False, book_list, author

def borrowBook(name, book_name, author):
    file = os.path.dirname(__file__)
    with open(file + '/../data/Return.csv', mode='a', newline='') as writeDatabase:
        writeDatabase = csv.writer(writeDatabase, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        data = [name, book_name, author]
        writeDatabase.writerow(data)
    r = csv.reader(open(file + '/../data/Borrow.csv', 'r'))
    lines = list(r)
    for i in range(len(lines)):
        if book_name == lines[i][0]:
            lines[i][2] = str(int(lines[i][2]) - 1)
    writer = csv.writer(open(file + '/../data/Borrow.csv', 'w'))
    writer.writerows(lines)

def returnBook(name, book_name, author):
    file = os.path.dirname(__file__)
    r_return = csv.reader(open(file + '/../data/Return.csv', 'r'))
    new_data = list(r_return)
    for i in range(len(new_data)):
        if name == new_data[i][0] and book_name == new_data[i][1] and author == new_data[i][2]:
            new_data.remove(new_data[i])
            break
    r_out = csv.writer(open(file + '/../data/Return.csv', 'w'))
    r_out.writerows(new_data)

    r = csv.reader(open(file + '/../data/Borrow.csv', 'r'))
    lines = list(r)
    for i in range(len(lines)):
        if book_name == lines[i][0]:
            lines[i][2] = str(int(lines[i][2]) + 1)
    writer = csv.writer(open(file + '/../data/Borrow.csv', 'w'))
    writer.writerows(lines)
