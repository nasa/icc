# Style Guidelines for *icc-sim-matlab*
Good practices for development of *icc-sim* modules. 
 
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
- __handles__: mixed case with leading capital [*NameOfHandle*]

Prefixes: 
- __quantities__ : use prefix "n_" when keeping track of how many of something you have  [*nSpacecraft*]
- **h_%% : handles 

## Structure 
- Aside from the *demos* directory, which contains various "main" scripts, scripts should be avoided. The functionality of the modules should comprise functions and classes. Scripts should never call other scripts.
- Global variables and frequently used input/output variables (e.g. spacecraft state array) should be indicated in the README. In the case of arrays, it should be indicated how the data is arranged. 
- Every file should contain a short description of what it does at the top.  
- Time should correspond to rows

## Units 
Unless otherwise specified, all functions should use the following units ... [TBD]

## Other 
Generally full names are preferred over symbols or abbreviations. e.g. *current_velocity* not *v_curr*