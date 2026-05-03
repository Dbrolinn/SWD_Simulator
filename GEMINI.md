# **SWAID Simulator: Coding Standards and Hard Rules**

This document outlines the strict coding standards and conventions to be used during the refactoring of the SWAID Chladni Simulator. All code submitted for review must adhere to these guidelines.

## **1\. Base Standard**

The project strictly follows the **C++ Google Style Guide**. Any ambiguity in the rules below should be resolved by referencing the official Google documentation.

## **2\. Naming Conventions**

* **Variables (Local and Parameters):** snake\_case (e.g., max\_iterations, frequency\_hz).  
* **Class Members (Variables):** snake\_case\_ with a trailing underscore (e.g., resolution\_, physics\_).  
* **Functions and Methods:** snake\_case (e.g., calculate\_mode\_frequency(), repair\_layout()).  
* **Classes and Structs:** CamelCase (e.g., PhysicsEngine, SimulationContext).  
* **Constants and Enums:** kCamelCase (e.g., kMinClearanceMeters, kGForce).  
* **Namespaces:** snake\_case (e.g., chladni).

## **3\. Formatting & Indentation**

* **Indentation:** 2 spaces per indent level. Do NOT use tabs.  
* **Line Length:** Maximum 100 characters per line.  
* **Braces:** Open braces { must be on the same line as the statement (if, for, while, class, function).  
* **Spacing:** Use spaces around binary operators (x \= y \+ z) and after commas.

## **4\. Documentation (Doxygen)**

All headers (.h files) must be fully documented using Doxygen style block comments /\*\* ... \*/.

* **Files:** Every file must have a @file and @brief tag at the top.  
* **Classes/Structs:** Must include a @class or @struct tag and a @brief description.  
* **Functions:** Must include @brief, @param (for each parameter), and @return (if non-void).

Example:  
/\*\*  
 \* @brief Strictly validates power feasibility to ensure acceleration \>= 1G.  
 \* @param frequency The resonance frequency in Hz.  
 \* @param ctx The current simulation context and hardware limits.  
 \* @return True if the pattern can be formed under 25W, false otherwise.  
 \*/  
bool validate\_power(double frequency, const SimulationContext& ctx);

## **5\. Memory Management & Pointers**

* **Raw Pointers:** The use of raw new and delete is strictly forbidden unless absolutely necessary for specific OpenGL C-API integrations.  
* **Smart Pointers:** Use ::std::unique\_ptr for exclusive ownership and ::std::shared\_ptr for shared ownership.  
* **Pass by Reference:** Always pass complex objects and strings by const reference (e.g., const std::string& name) to avoid unnecessary copying.

## **6\. Standard Library and Namespaces**

* **No Global Namespace Pollution:** Using directives like using namespace std; are strictly forbidden anywhere in the codebase.  
* **Standard Library Prefix:** As established in the current codebase, standard library types must be explicitly prefixed with ::std:: in header files to ensure global namespace resolution (e.g., ::std::vector\<double\>, ::std::string).

## **7\. Error Handling & State Validation**

* Return early. Avoid deep nesting of if statements.  
* Use ::std::optional for return types that might fail to compute a valid result (e.g., when a frequency cannot be found).  
* Assert physical constraints early. If a transducer coordinate violates kMinClearanceMeters, handle it immediately rather than propagating invalid geometry through Eigen matrices.

## **8\. Approved Third-Party Libraries**

* **Math:** Eigen3 (Matrix operations).  
* **UI:** Dear ImGui, ImPlot.  
* **JSON:** nlohmann/json (Must be used for all configuration serialization).  
* **Image Export:** stb\_image / stb\_image\_write.