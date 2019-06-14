# Style Guidelines for *icc-dev*
Good practices for development of *icc-dev* modules. 
 
## File and Folder Name Format
- __directories/modules__: all lowercase, seperated by underscore [*name_of_module*]
- __functions__: all lowercase (except names), seperated by underscore [*name_of_function.m*]
- __scripts__: mixed case starting with capital, seperated by underscore [*Name_Of_Script*] 
- __classes__: mixed case starting with capital [*NameOfClass*]

## Variable Name Format 
Variable Types:
- __numerical arrays__: all lowercase, seperated by underscore [*name_of_variable*]
- __logical variables__: mixed case starting with lowercase [*nameOfLogicalVariable*]
- __flag variables__: prefix flag + mixed case starting with lowercase [*flag_nameOfFlag*]
- __PATH variables__: all uppercase, seperated by underscore [*PATH_NAME*]
- __objects, cells, class properties, and structs__: mixed case, starting with uppercase [*NameOfStruct*]

Prefixes: 
- n_  : quantity of something  [*n_spacecraft*]
- i_ : indicies [*i_spacecraft*]
- h_ : handles [*h_orbitPlot*]

## General Guidelines/ Other 
- Aside from the *examples* directory, which contains various "main" scripts, scripts should be avoided. The functionality of the modules should comprise functions and classes. Scripts should not call other scripts.
- Every file should contain a short description itself at the top.  
- Functions should indicate the meaning of all input and output variables. See the *template_function* for an example of this. 
- Generally full names are preferred over symbols or abbreviations. e.g. *current_velocity* not *v_curr*
- Modules should be self contained if at all possible 
