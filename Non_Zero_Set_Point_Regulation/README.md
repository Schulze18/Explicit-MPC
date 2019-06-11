# Non-zero set-point regulator to the quadrotor Iris 3DR 
You cand copy or edit the files *iris_mimo_control_deltaU_generator.m* and *iris_mimo_implementation.m* with the data from your system.

*modelo_nlinear_iris.slx* is a *Simulink* simulation file that you can test your controllers for this quadrotor. First you need to convert the set of regions to a struct file (use *regions_cell_to_struct*), then change the loaded file in the *InitFcn* callback from block *Controlador_mimo/Busca_Regiao*.