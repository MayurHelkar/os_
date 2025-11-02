<details>
<summary>Process</summary>
  
* *Instance* of the program execution.
* Progress in a sequential fashion.

| Program | Category | Process |
| ------- | -------- | ------- |
| *Passive* | Entity | *Active* |
| Process ID (PID)<br>Process State<br>Program Counter<br>CPU Registers<br>Priority<br>Memory Management Information<br>I/O Status Information<br>Accounting Information<br>Pointers<br>Parent Process ID (PPID)<br>Security/Protection Attributes | Attributes | |
| **Four**<br>*Text*<br>*Data*<br>*Heap*<br>*Stack* | Memory sections | **Two**<br>*Text*<br>*Data*|
</details>

Process memory is divided into four sections for efficient working
| Memory Section | Description |
| -------------- | ----------- |
| *Text*/ *Code* | Compiled program code<br>Read in from non-volatile storage during launch<br>Read-only permission<br>Size :- Depends  on the number of instructions and the program’s complexity<br>Example :- [`.exe` Windows, `.elf` Linux] |
| Data | Consists of Gobal and Static variables<br>Variables retain their values throughout program execution<br>Allocated and initialized prior to executing the main<br>Two types --<br>`bss` Block Started by Symbol - unitialized<br>initialized |
