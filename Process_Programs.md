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
| *Text* | Compiled program code<br>Rread in from non-volatile storage during launch [`.exe` Windows, `.elf` Linux]
