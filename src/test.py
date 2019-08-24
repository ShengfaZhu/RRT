class Student(object):
    def __init__(self, id, courses):
        self.id_ = id
        self.courses_ = courses

if __name__ == '__main__':
    a = Student(1, [1.1, 1.2, 1.3])
    b = Student(2, [2.1, 2.2, 2.3])
    a, b = b, a
    print("a is ")
    print(a.id_)
    print(a.courses_)
    print("b is ")
    print(b.id_)
    print(b.courses_)



