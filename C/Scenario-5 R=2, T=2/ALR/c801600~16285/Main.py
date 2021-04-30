#coding=utf-8
"""
Solve the Reliability-oriented GAP by ADMM-based method.
Date:2021-3-5
version: 1.0
Author: Maocan Song.
"""
from Model import ADMM
import time
def main():
    time_start=time.time()
    mod=ADMM()
    mod.g_solving_the_GAP_by_LR_heuristics()
    time_end=time.time()
    spend_time=(time_end-time_start)
    mod.g_output_the_results(spend_time)
    print("Spend time: {} min".format(round(spend_time/60,3)))

if __name__=="__main__":
    main()