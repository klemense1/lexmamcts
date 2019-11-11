set terminal pdf size 8,5
set output "/tmp/selection_policy_compare.pdf"

#set size 1,0.85

#set logscale y
#set logscale x
set grid mxtics mytics xtics ytics lt 1 lc rgb 'gray70', lt 1 lc rgb 'gray90'

set xlabel "# samples"
set ylabel "L2 norm of |V*(t=0)-V(t=0)|"

stats '/tmp/policy_comp.dat' u 2 name 'uct'
stats '/tmp/policy_comp.dat' u 3 name 'pareto'
stats '/tmp/policy_comp.dat' u 4 name 'slack'
stats '/tmp/policy_comp.dat' u 5 name 'egreedy'

set label 1 sprintf("UCT mean: %.2f\nApproximate pareto-optimal UCT mean: %.2f\nUCT with 95%% confidence bounds mean: %.2f\nEpsilon-Greedy UCT mean: %.2f", uct_mean, pareto_mean, slack_mean, egreedy_mean) at graph 0.02,0.97

plot "/tmp/policy_comp.dat" u 1:2 w linespoints t "UCT", \
"" u 1:3 w linespoints t "Approximate pareto-optimal UCT",\
"" u 1:4 w linespoints t "UCT with 95% Confidence bounds",\
"" u 1:5 w linespoints t "Epsilon-Greedy UCT"
