#include "../../head/neighbor-discovery.h"
module CkndManagerP
{
	provides interface CkndManager;
}

implementation
{ 
#undef FREENODE
     #define FREENODE  255          //空闲位置

	 cknd_tab_t NodesTable[CKNDSETSIZE];  
	// uint8_t setsz=0;//使用的节点数量

     command void CkndManager.init(){
         uint8_t i;
         for(i=0;i<CKNDSETSIZE;i++)
            NodesTable[i].nodeId=FREENODE;
     }
     command uint8_t CkndManager.length(){
         uint8_t i;
         uint8_t nodenum;
         for(i=0;i<CKNDSETSIZE;i++){
              if(NodesTable[i].nodeId!=FREENODE)
                 nodenum++;
         }
         return nodenum;
     }
     /*
	 *  返回节点在数据的位置索引，没有找到 返回最大节点号
	 */
	 uint8_t findNode(uint8_t nodeid){
             uint8_t i;
	     for(i=0;i<CKNDSETSIZE;i++){
		    if(NodesTable[i].nodeId==nodeid)
			   return i;
		 }
		 return CKNDSETSIZE;
	 }
     command cknd_tab_t *CkndManager.find(uint8_t nodeid){
	     uint8_t in=findNode(nodeid);
		 if(in==CKNDSETSIZE) return NULL;
		 return &NodesTable[in];
	 }
     //存在该节点或者没有空间的节点都返回NULL
     command cknd_tab_t *CkndManager.insert(uint8_t nodeid){
            uint8_t in=findNode(nodeid);
            if(in!=CKNDSETSIZE) return &NodesTable[in];//这个是找到节点的意思，节点在表中,返回节点所在位置

            in=findNode(FREENODE);//找到第一个空的位置
            
	    if(in==CKNDSETSIZE) return NULL;//如果没有找到，退出
            return &NodesTable[in];//返回空位置
     }

     command cknd_tab_t* CkndManager.delete(uint8_t nodeid){
           uint8_t in=findNode(nodeid);
           if(in==CKNDSETSIZE) return NULL;//没有找到这个节点
           NodesTable[in].nodeId=FREENODE; //找到之后直接将这个节点清空
           return &NodesTable[in];//返回节点在表中的空位
     }
//这个函数还没有用
      command cknd_tab_t* CkndManager.traverse(cknd_tab_t* in){
	            cknd_tab_t *end=NodesTable+CKNDSETSIZE;
				cknd_tab_t *start=in?in:NodesTable;
				while(start!=end){
					if(start->nodeId!=FREENODE)
					    return start;
					start++;
				}
                return NULL;//表明已经到表尾了
	   }
}
