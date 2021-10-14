# AzA_Avionics  
Code of the Avionics team at the Arizona Autonomous Vehicles Club  
  
Python code that handles server communications and autonomous flying of the club's plane  
  
Status:  
Server communications - Not handled  
Decision making - Not handled  
Pathfinding - Only 2d straight line pathfinding  

fig_files <- list.files(pattern = "jpg$") # change to png$ as needed!
writeLines(paste0(fig_files, "\n",
                  "![`", fig_files, "`](", fig_files, ")\n\n"), "README.md")
