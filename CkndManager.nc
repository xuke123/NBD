#include "../../head/neighbor-discovery.h"
typedef struct cknd_tab_t cknd_tab_t;

interface CkndManager{
    command void init();
    command uint8_t length();
    command cknd_tab_t* find(uint8_t nodeid);
    command cknd_tab_t* insert(uint8_t nodeid);
    command cknd_tab_t* delete(uint8_t nodeid);
    command cknd_tab_t* traverse(cknd_tab_t* ne_index);
}
