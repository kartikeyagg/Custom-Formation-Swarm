class RailwayForm:

    time = "night"

    def __init__(self,name,subunit):
        print("Employee is created")
        self.name = name
        self.subunit = subunit
        self.form_type  = "railwayForm"


    def printData(slf):  # see here slf is used instead of self
        print("Name is "+str(slf.name))
        print("Train is "+str(slf.train))

    # @staticmethod
    def greet(self):
        print("good moring manayavar")
        # print( "Name is {self.train}")
    def print_time(self):
        print(self.time)


harryApplication = RailwayForm("Kartikey","Delhi")
# harryApplication.name = "Kartikey_A"
harryApplication.train = "Rajdhani express"
harryApplication.printData()
harryApplication.print_time()
harryApplication.greet()
harryApplication.time = "day"
harryApplication.print_time()
