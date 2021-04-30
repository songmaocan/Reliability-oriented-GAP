$ include "C:\OneDrive\OneDrive - 东南大学\科研\备份文件夹\科研\博士研究进展汇报\2020-6(ADMM_VRP, GAP)\An ADMM based method for GAP\Python\A-E\Gap-A\LR\a05100~1698\GAMS_input_file.txt";

variable z;
binary variable x(i,j);
equation
object
job(j)
resource(i)
;
object.. z =e= sum(i,sum(j,c(i,j)*x(i,j)));
job(j).. sum(i,x(i,j)) =e= 1;
resource(i).. sum(j,a(i,j)*x(i,j)) =l= b(i);
model gap /all/;
*Note:maximize for small example
gap.optCR=0
solve gap minimizing z using MIP;
