"""
Read gap data and output gams file
Date:2021-3-5
"""
class Read_GAP_data:
    def __init__(self):
        self.file="input.txt"
        self.g_number_of_agents=0
        self.g_number_of_jobs=0
    def read_data(self):
        with open(self.file,"r") as fl:
            lines = fl.readlines()
            #data-1:number of agents and jobs
            line_1=lines[0].strip().split()
            self.g_number_of_agents=int(line_1[0])
            self.g_number_of_jobs=int(line_1[1])
            #data-2:resouce
            # line_1 = lines[-1].strip().split()
            # self.g_agent_resouce_list=[0]*self.g_number_of_agents
            # for m in range(self.g_number_of_agents):
            #     self.g_agent_resouce_list[m]=int(line_1[m])
            #data-3:cost
            Flag=0
            self.cost_matrix=[]
            self.resouce_matrix=[]
            empty_list = []
            for line in lines[1:]:
                line_1 = line.strip().split()
                if len(empty_list)==self.g_number_of_jobs:
                    if Flag<self.g_number_of_agents:
                        self.cost_matrix.append(empty_list)
                        empty_list=[]
                        Flag+=1
                    else:
                        self.resouce_matrix.append(empty_list)
                        empty_list = []
                        Flag += 1
                for item in line_1:
                    empty_list.append(int(item))
                    # empty_list.append(int(item))
                if len(empty_list)==self.g_number_of_agents and line==lines[-1]:
                    self.g_agent_resouce_list=empty_list
        print("read data is finished!")
# put data in list
        self.agent_list=[]
        self.job_list=[]
        for m in range(self.g_number_of_agents):
            agent=Agent()
            agent.agent_id=m
            agent.resource=self.g_agent_resouce_list[m]
            agent.cost_list_each_job=self.cost_matrix[m]
            agent.resource_list_each_job=self.resouce_matrix[m]
            self.agent_list.append(agent)

        for j in range(self.g_number_of_jobs):
            #initialize the multipliers
            multiplier_for_j=10000
            for m in range(self.g_number_of_agents):
                if multiplier_for_j>self.cost_matrix[m][j]:
                    multiplier_for_j = self.cost_matrix[m][j]

            job=Job()
            job.job_id=j
            job.multiplier=multiplier_for_j*(-1)
            job.multiplier_ALR=multiplier_for_j*(-1)
            self.job_list.append(job)
        #output GAMS file
        self.output_GAMS_file()
        return self.g_number_of_agents,self.g_number_of_jobs,self.agent_list,self.job_list

    def output_GAMS_file(self):
        with open("GAMS_input_file.txt", "w") as fl:
            fl.write("set j job /0*{}/;\n".format(self.g_number_of_jobs - 1))
            fl.write("set i machine /0*{}/;\n".format(self.g_number_of_agents - 1))

            fl.write("parameter c(i,j)/\n")
            for i in range(self.g_number_of_agents):
                agent = self.agent_list[i]
                for j in range(self.g_number_of_jobs):
                    cost = agent.cost_list_each_job[j]
                    fl.write(str(i) + ". " + str(j) + " " + str(cost) + "\n")
            fl.write("/;\n")

            fl.write("parameter a(i,j)/\n")
            for i in range(self.g_number_of_agents):
                agent = self.agent_list[i]
                for j in range(self.g_number_of_jobs):
                    resource = agent.resource_list_each_job[j]
                    fl.write(str(i) + ". " + str(j) + " " + str(resource) + "\n")
            fl.write("/;\n")

            fl.write("parameter b(i)/\n")
            for i in range(self.g_number_of_agents):
                resource = self.agent_list[i].resource
                fl.write(str(i) + " " + str(resource) + "\n")
            fl.write("/;\n")

# a=Read_GAP_data()
# a.read_data()
class Agent:#machine
    def __init__(self):
        self.agent_id=0
        self.resource=0
        self.cost_list_each_job=None
        self.resource_list_each_job=None


class Job:
    def __init__(self):
        self.job_id=0
        self.multiplier=0
        self.multiplier_ALR=0 #三项
