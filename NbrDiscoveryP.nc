#include "../../head/neighbor-discovery.h"
#include "../../head/neighbor-manager.h"
#include "pwmac2.h"
#include "../../head/list.h"

#include <string.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
//#include "Debug.h"

module NbrDiscoveryP{
	uses{
		interface Timer<TMilli> as checkTimer;
		interface Timer<TMilli> as radioTimer;                

		interface sendImmediate;
		interface AMSend  as ListenBeacon; 
		interface Receive as NotifyData;
		interface CC2420PacketBody;
		interface LocalTime<TMilli> as localtime;
		interface pseudoWakeupTimeCompute;
		interface beaconAppend;
		interface neigList as NbrManager;
		interface CkndManager;
		interface whetherInDC;
		interface Packet;
		interface Boot;
		interface PWMacListener;

		interface receiveTime;
		interface radioConfig;
	}
}
implementation{
	//测试宏
#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define TESTCOORDINATION 0

#define TIMESTAMP 0
#if TIMESTAMP
uint32_t BeaconApend,BeaconApstart;
uint32_t snoopFilterstart,snoopFilterl1,snoopFilterend;
uint32_t obtainPstart,obtainPrend;
uint32_t nodecheckTime;
#endif

	//节点唤醒时间管理单元
	LIST(wakeup_manager);
	//节点超时和正需要的节点
	LIST(service_manager);
	//潜在节点未检查的节点集合
	LIST(uncknode_manager);

	void fresh_checktimer();
	void obtainPotentialnode(void *payload,int32_t offsettime);
	uint32_t calRendezvousTime(struct neig_tab_t *nodeitem,uint8_t count);//添加一个字段，表示节点当前邻居节点interval起点
	uint32_t calPotentialCktime(struct cknd_tab_t *nodeitem);
	uint8_t FillNeighborSet_withoutTime(void *set,uint8_t sz);
	void  TimeoutButNoDealNode(struct sche_tab_t *iterm); //节点超时了，但是未来的急处理的节点的办法

	//链表操作
	struct list{
		struct list* next;
	};
	void   list_init(list_t list);
	void * list_head(list_t list);
	void * list_tail(list_t list);
	void * list_pop (list_t list);
	void   list_push(list_t list, void *item);
	void   list_add(list_t list, void *item);
	void   list_remove(list_t list, void *item);
	void   list_insert(list_t list, void *previtem, void *newitem);
	void * list_item_next(void *item);
	//将节点插入排序节点的合适的位置
	void insertNode(list_t list,struct sche_tab_t *node);

	/*const uint8_t true=1;
	  const uint8_t false=0;*/
	enum{
		false=0,
		true=1,       
	};
	//---------------------------------------------------------
	message_t NotifyData_msg;
	message_t Listen_msg;
	uint8_t NotifyDest;
	uint32_t rcvTime;  


	void generateNotifyData(int32_t offsetTime);

	event void Boot.booted(){
		list_init(wakeup_manager);
		list_init(service_manager);
		list_init(uncknode_manager);
		call CkndManager.init();//检查表清空,因为没有在这个组件事先实现boot时间
#if TESTCOORDINATION
		printf("test COORDINATION starting...\r\n");
#endif
	}
	//---------------------- 参数发送和接收-------------------
	event void receiveTime.get(uint32_t ct,uint8_t type){   //we get pkt recevied time,event should be placed in all of the places releated to neigInsert 
		rcvTime = ct;
	}
	/*
	 *  处理发送邻居节点发现包的情况
	 */
	event void sendImmediate.requestSendImmediate(message_t *msg){    //这个notifyData不是本地的数据结构吗
		//这里判断是否立刻发送
		if(msg==&NotifyData_msg)//只要是邻居节点发现的数据包，都需要立刻发送
			call sendImmediate.setSendImmediate(true);
	}
	//----------------
	/*
	 *产生邻节点发现包
	 */
	void generateNotifyData(int32_t offsetTime){
		struct NotifyData *pload=(struct NotifyData *)call Packet.getPayload(&NotifyData_msg,sizeof(struct NotifyData));
		myPseudoPara_t myInfo;
		pload->type=3; //什么字段都可以
		pload->offsetTime= offsetTime;     //根据第一个beacon的时间                

		myInfo=call pseudoWakeupTimeCompute.getMyPseudoPara();
		pload->para.lcg_a=myInfo.lcg_a; 		
		pload->para.lcg_c=myInfo.lcg_c; 	
		pload->para.lcg_m=myInfo.lcg_m;
		pload->para.random_state=(uint16_t)myInfo.random_state; 	
		pload->para.nextWakeupTime=myInfo.nextWakeupTime; 
	}
	//---------------------------------------------------------------------------------------------
	/*
	 *  还有一种节点收到邻节点发现包的处理情况
	 */
	event message_t* NotifyData.receive(message_t *msg,void *payload,uint8_t len){ 

		cc2420_header_t *header;
		struct neig_tab_t *neig;
		struct NotifyData *pload;
		uint32_t preBeaconTime;
		struct PseudoPara* neigInfo;

		if(call whetherInDC.isDC()==false) return msg;//初始化直接退出，因为不可能能收到邻节点发现包

		header=(cc2420_header_t *)call CC2420PacketBody.getHeader(msg); 
		neig=call NbrManager.find(header->src);
		pload=(struct NotifyData *)payload;  
		if(!neig){
			printf("L%u: node%u be inserted\r\n",__LINE__,neig->nodeId);
			//计算节点上次发送beacon时间，注意打包的其实是节点下次发送beacon的时间
			neigInfo=&(pload->para);
			preBeaconTime=neigInfo->nextWakeupTime;//-neigInfo->random_state;

			neig=call NbrManager.neigInsert(header->src,neigInfo,preBeaconTime-pload->offsetTime,preBeaconTime);//注意此时获取参数，其实是计算过一遍的了

			if(!neig) return msg;// 节点在没有插入节点的时候应该退出

			neig->check_time=calRendezvousTime(neig,0);//这一次应该不计算
			neig->type=KNOWNNODE;    //已经知道对方参数了
			neig->link_status=TWOWAY;//对方收到我的beacon，我收到对方的邻居节点发现数据，那么此时节点应该是双向的关系
			insertNode(wakeup_manager,(struct sche_tab_t *)neig);//加入链中，立刻去检查节点

			fresh_checktimer();
			//printf("L%u: node%u fresh_checktimer\r\n",__LINE__,TOS_NODE_ID);
		}
		return msg;
	}

	//--------------------------------------计算预约点时间---------------------------------------------
	/*
	 * 计算节点的预约点时间：这段时间计算还存在问题！！！
	 *调用时刻：节点在每次服务完一个节点后，都会将节点时间计算出来，
	 *         写入在邻居节点的表项中去（当然重新加入链表后，这个时间就会排队）
	 * 这里根据表项的伪随机参数，和当前beacon的值，设置下次预约点beacon, count  为1所以收到beacon，那么下次就是0 预约点
	 */
	uint32_t calRendezvousTime(struct neig_tab_t * item,uint8_t count){//注意这个函数并没有修正偏差的！！！！
		//只要连续计算出count次时间间隔就好, 写不写check_time字段，该函数不决定
		uint32_t interval=0;
		uint32_t nowinterval=item->last_random_state;//这个Interval 算最新的吧，因为count值收到的前提是我收到count，收到count的
		uint32_t nodeTime;
                /*+item->offset_b*/
		nodeTime=item->check_time+ADVANCE2CHECK;//第一次是插入的节点时，节点收到beacon的那个时间点，此时是最准的，随后，之后就是迭代的计算的值了，注意这个时间包含提前唤醒的时间
		//的条件，在占空比的情况下，是我算对了单钱的Interval  20170818添加时钟漂移neig->offset_b
                //节点上次检查的时间
		while(count--){
			nowinterval= (item->lcg_a * nowinterval +  \
					item->lcg_c) % item->lcg_m + MIN_INTERVAL;
			interval+=nowinterval;
		}
		item->last_random_state=nowinterval;
		interval=nodeTime+interval-ADVANCE2CHECK;//自己时间
		//邻居节点的时间了，注意interval中有提前唤醒的时间，所以再加回去
		//printf("L%u: node%u Time wake %lu,@ %lu\r\n",__LINE__,item->nodeId,item->b+interval+item->offset_b+ADVANCE2CHECK,interval);
              //  printf("L%u: node%u Time wake %lu,@ %lu\r\n",__LINE__,item->nodeId,item->b+interval+ADVANCE2CHECK,interval);//去掉offset_b offset-b只修正上次计算的的值
		if(interval <call localtime.get()){//计算的时间小于节点当前时间，出错的可能比较大，暂时不考虑这种情况吧
		}
		return interval;
	}

	//------------------------------------------------OK（未测）-------------------------------------------------------
	/*
	 *填充邻节点表集合（带时间），直接遍历一遍链表即可（无需计算时间）
	 */
	uint8_t FillNeighborSet_withTime(void* set,uint8_t sz){
		uint8_t index=0;
		struct Neighbor* pset=(struct Neighbor*)set;
		struct sche_tab_t* te=list_head(wakeup_manager);
		for(;te!=NULL&&index<sz;te=list_item_next(te)){
			if(te->type==UNKNOWNNODE) continue;//只装已知的节点
			pset[index].nodeId=te->nodeId;
			//简单期间，直接将节点下次唤醒的时间的加进入，存在的问题，可能即使收到的beacon不带参数
			//解决办法当然，大不了用type标识，需要立刻检查的节点，直接把这部分节点重新算一遍预约时间就得了
			pset[index].offsetTime=te->check_time;
			index++;
			//if(index>=sz) break;//已经装不下了
		}
                //还得检查service_manager
                te=list_head(service_manager);
		for(;te!=NULL&&index<sz;te=list_item_next(te)){
                       if(te->type==UNKNOWNNODE) continue;//只装已知的节点
			pset[index].nodeId=te->nodeId;
			//简单期间，直接将节点下次唤醒的时间的加进入，存在的问题，可能即使收到的beacon不带参数
			//解决办法当然，大不了用type标识，需要立刻检查的节点，直接把这部分节点重新算一遍预约时间就得了
			pset[index].offsetTime=te->check_time;
			index++;   
                }
		// index就是邻节点表的终止标志
		if(index<sz) pset[index++].nodeId=FREENODE;
		//printf("L%u: node%u pack RENDEZ %u,%u,%u,...\r\n",__LINE__,TOS_NODE_ID,pset[0].nodeId,pset[1].nodeId,pset[2].nodeId);
		return index;
	}
	//--------------------------------------------------------------------------------------------------------------
	/*
	 *在节点的Beacon中payload设置参数
	 * 四种类型：
	 0：beacon为初始化beacon (现在已经没有了！！！20170808)
	 1：不带邻节点表的占空比beacon
	 2：带邻居节点表的占空比beacon,随后beacon不带
	 3：带邻居节点表并且随后的beacon也带
	 * 注：简化版本暂时不考虑邻居节点集合被划分到多个beacon的情况，
	 *     那么带邻居节点集合的节点的否是类型2，unfinished字段当然也只能是0
	 */
	event void beaconAppend.requestBeaconAppend(){
		static uint8_t count=0,unfinished=0;
		// bool is_DC=call whetherInDC.isDC();
#if TIMESTAMP
BeaconApstart=call localtime.get();
#endif

		count++;    
		//count%=MAX_BEACONCOUNT;
		//20170808删除， 这里没有初始化beacon
		/*
		 * 工作在占空比：两个因素：节点的count值，以及节点预约点beacon是否全部被发送出去
		 */
		if(count>MAX_BEACONCOUNT) count=0;
		if(count==0){  
			struct rendezBeaconP renBeacon;
			myPseudoPara_t myInfo;
			renBeacon.type=RENDEZ;renBeacon.srcId=TOS_NODE_ID;renBeacon.count=count;
			//strcpy(renBeacon->para,call pseudoWakeupTimeCompute.getMyPseudoPara());
			myInfo=call pseudoWakeupTimeCompute.getMyPseudoPara();

			renBeacon.para.lcg_a=myInfo.lcg_a; 		
			renBeacon.para.lcg_c=myInfo.lcg_c; 	
			renBeacon.para.lcg_m=myInfo.lcg_m;
			renBeacon.para.random_state=(uint16_t)myInfo.random_state; 	
			renBeacon.para.nextWakeupTime=myInfo.nextWakeupTime; 

			renBeacon.flag=2;
			FillNeighborSet_withTime(renBeacon.set,SETSIZE);

			if(call beaconAppend.setBeaconAppend((void*)(&renBeacon),sizeof(renBeacon))==ESIZE){
				printf("beaconpload overflow,sz is %u\r\n",sizeof(renBeacon));
			}; 
#if TIMESTAMP
                        BeaconApend=call localtime.get();
#endif
			//printf("L%u: node%u sendRendezBeacon!\r\n",__LINE__,TOS_NODE_ID);
#if TIMESTAMP
   //  printf("TIMESTAMP:BeaconApstart,BeaconApend %lu %lu\r\n",BeaconApstart,BeaconApend);
#endif     
			unfinished=0;
		}else if(unfinished){
			//暂无 这里应为RENDEZ_SUC类型的处理函数
		}else{//只发最简单的beacon就好
			struct BaseBeaconP base;
			base.type=BASE;base.srcId=TOS_NODE_ID;base.count=count;
			call beaconAppend.setBeaconAppend((void*)&base,sizeof(base));
		}

	}

	//--------------------------------------------------OK（未测）---------------------------------------------
	/*
	 *  节点侦听已知节点和未知节点的处理情况（这里只针对未能收到beacon的情况，收到beacon在隔壁Filter中，
	 调用AMSend返回：EBUSY:上一次数据还未发送完成，初始化还未完成。  FAIL:目的节点不在邻节点表中
	 调用附加BEACON返回:ESIZE：发送数据包长度超过124；否则为SUCCESS。
	 */
	event void ListenBeacon.sendDone(message_t* msg, error_t error){  //FAIL：底层底层done事件里失败，没收到beacon，没收到ACK；EBUSY:提交发送任务的失败，节点正处于接收状态，或者将要发beacon
		/*如果是发空包侦听   定义类型 LISTENBEACON*/
		//struct sche_tab_t *te,*tenext;
		// struct cknd_tab_t  *tc;
		//struct neig_tab_t *tn;
		//cc2420_header_t *header=(cc2420_header_t *)call CC2420PacketBody.getHeader(msg);
		/*1.成功就直接退出*/
		// 因为在收到的beacon的处理函数 会处理这部分逻辑

		//printf("ListenBeacon.sendDone is toggle!\r\n");    //现在只有数据监听了            

		if(error==SUCCESS) {printf("send data is OK");}
		else{
			switch(error){ 
				case EBUSY:printf("L%u:EBUSY\r\n",__LINE__);break;
				case FAIL :printf("L%u:FAIL\r\n" ,__LINE__);break;
				case ESIZE:printf("L%u:ESIZE\r\n",__LINE__);break;
				default   :printf("L%u:FAIL?\r\n",__LINE__);
			}	
		};

	}
	//---------------------------------OK(未测)----------------------------------------------
	/*
	 * 超时了，但是没有时间服务的节点处理办法，并决定节点加入哪一个集合
	 * 已知节点：去节点下一个beacon去听    未知节点：算没有听到对方的beacon
	 */
	void  TimeoutButNoDealNode(struct sche_tab_t *te){
		struct cknd_tab_t *cte;
		if(te->type==UNKNOWNNODE){
			cte=(struct cknd_tab_t*)te;
			cte->handle_times=0;
			insertNode(uncknode_manager,(struct sche_tab_t*)cte);//添加到未检查的节点集合中，注意这个的也是按照时间排序的
			//printf("L%u: node%u UNKNOWN but beyond time undeal\r\n",__LINE__,te->nodeId);
		}else if(te->type==KNOWNNODE){
			//计算节点下次预约点时间
			te->check_time=calRendezvousTime((struct neig_tab_t*)te,3);//立刻去听,对于对于初始化节点同样适用,时间有点短，设置为三个、
			//printf("L%u: node%u calRendez at TimeoutButNoDealNode\r\n",__LINE__,te->nodeId);
			insertNode(wakeup_manager,te);
		}
	}

	//-------------------------------------------------------------------------------------------
	/*
	 *节点超时任务，任务移除服务链表中所有的节点到合适的位置
	 */
	event void radioTimer.fired(){
		struct sche_tab_t* te,*tenext;
		struct neig_tab_t* tn;
		// 处理开放radio时间内没有得到服务的节点
		//printf("radioTimer fire\r\n");
		/*if(call radioConfig.isRadioActive()==false){
		  printf("radio is be closed!\r\n");
		  }*/
		te=list_head(service_manager);
		/*if(te==NULL)
			printf("and service_manager is NULL\r\n");*/
		for(;te!=NULL;te=tenext){
			tenext=list_item_next(te);//先记录下一个节点

			list_remove(service_manager, te);

			if(te->type==KNOWNNODE){
				/*类型0 已知的节点
				 *  使用空包调用updateNeighbor，计算节点下次检查时间
				 * （1.如果节点还在，失败了应该是立刻检查下一个beacon才对
				 *   2.如果节点丢失，将节点从两条severce和wakeup两条链表中移除）
				 */
				tn=(struct neig_tab_t*)te;
				tn->fail_time++;
				if(tn->fail_time>3){
					call NbrManager.delete(tn->nodeId);
					printf("L%u: node %u is deleted #check_fail\r\n",__LINE__,tn->nodeId);
					tn=NULL;//节点被删除
				}
				if(tn==NULL){//可以删除了
					;//注意updateNeighbor 函数会将节点删除
				}else{
					printf("L%u: node%u is failcheck\r\n",__LINE__,tn->nodeId);
					te->check_time=calRendezvousTime(tn,1);
					insertNode(wakeup_manager,te);//应该立刻去检查节点
				}
			}else if(te->type==UNKNOWNNODE){
				/*类型1 潜在节点 处理办法相同
				 * 节点没有收到beacon

				 * 将节点从server_manager 移动到 uncknode_manager，设置计数值为0
				 * 节点在收到未知节点的beacon，但是节点并没有发送数据成功
				 */
				((struct cknd_tab_t*)te)->handle_times=0;
				insertNode(uncknode_manager,te);
				printf("L%u: node%u UNKNOWN but rece nothing\r\n",__LINE__,te->nodeId);
			}
		}
		//随后处理第二个节点

		fresh_checktimer();
		//printf("L%u: node%u fresh_checktimer\r\n",__LINE__,TOS_NODE_ID);
			;//printf("L%u: node%u have no node check!\r\n",__LINE__,TOS_NODE_ID);	
#if TIMESTAMP
    printf("TIMESTAMP: snoopFilterstart %lu,snoopFilterl1 %lu,snoopFilterend %lu,obtainPstart %lu,nodecheckTime %lu\r\n",
                                             snoopFilterstart,snoopFilterl1,snoopFilterend,obtainPstart,nodecheckTime);
#endif

	}
	//-----------------------------------OK(未测)---------------------------------------------------
	/*
	 * 具体办法：节点超时后，依照节点type确定自身的任务类型,初始化过程，节点时不进入服务链的
	 */
	event void checkTimer.fired(){
		struct sche_tab_t* te;
		//struct neig_tab_t* tn;
		uint32_t uppertime;
		//uint32_t nextcktime;
		uint32_t nowTime;
		static int BusyCount=0;//记录底层忙的次数

		nowTime=call localtime.get();
		uppertime=nowTime+CHECKUPPERTIME;
		//如果是初始化的话
		//printf("checkTimer fire\r\n");
		if(call whetherInDC.isDC()==FALSE){//占空比节点检查初始化节点怎么办
			fresh_checktimer();
			//printf("L%u: node%u fresh_checktimer in init phase\r\n",__LINE__,TOS_NODE_ID);
			return;
		}
		//不断添加超时的节点的
		te=list_head(wakeup_manager);

		for(;te!=NULL;te=list_item_next(te)){
			if(te->check_time < uppertime){//SB了，这里应该是将小于上限的时间加入的其中，所以加上超时的节点，至少应该有一个节点
				list_remove(wakeup_manager, te);
				insertNode(service_manager,te); 
			}
		}
		/*----------------------------------------------
		  这里不合适，触发下一个检查任务的时间, 直接属性检查定时器就好：
		  1)节点未能打开radio去听，期间又一次5ms的时间缓冲
		  2）节点打开radio收到节点信息beacon，之后，处理完成
		  3）节点未收到信息
		  te=list_head(wakeup_manager);
		  nextcktime=te->check_time-nowTime;

		  call checkTimer.startOneShot(nextcktime);//设置节点下一个检查间隔
		  ------------------------------------------------*/

		if(call radioConfig.keepRadioActive(30)!=SUCCESS){
			printf("call radioConfig is error\r\n");
			call checkTimer.startOneShot(5);// 5ms后再调用一次，如果仍不行，按节点未收到beacon处理
			BusyCount++;
			if(BusyCount==2){//超过2次了，处理服务集合中所有的节点
				call radioTimer.startOneShot(1);
				BusyCount=0;		
			}else
				return; //小于3次就不断检查
		};
		/*  if(call radioConfig.isRadioActive()==false){
		    printf("radio is not open!\r\n");
		    }*/
		call radioTimer.startOneShot(30);
		te=list_head(service_manager);
		if(te)
			if(te->type==KNOWNNODE){
;
#if TIMESTAMP
				nodecheckTime=te->check_time+((struct neig_tab_t*)te)->b+ADVANCE2CHECK;//printf("L%u: node%u Time: KNOWN check @ %lu\r\n",__LINE__,te->nodeId,te->check_time+((struct neig_tab_t*)te)->b+ADVANCE2CHECK);//加上提前时间吧
#endif
}
			else // 
				printf("L%u: node%u Time: UNKNOWN check @ %lu\r\n",__LINE__,te->nodeId,te->check_time+ADVANCE2CHECK);//注意这个时间已经通过的邻居节点的偏移时间修正为本地时间
		else    ;

	}
	//-----------------------------------------------OK(未测)-----------------------------------------------
	/* 
	 * 使用情况：在唤醒管理中，每添加一个节点，就应该刷新检查定时器
	 * 具体办法：查看当前等待时间是否大于头节点的等待时间
	 * 1. 小于，直接退出
	 * 2. 大于，重新设定定时器的等待时间（当前节点等待时间是缩小的）
	 * ---------> 节点停掉定时器，重新设置时间吧
	 */
	void fresh_checktimer(){
		uint32_t nowtime,nextcktime;
		struct sche_tab_t* te,*tenext;
		nowtime=call localtime.get();
		te=list_head(wakeup_manager);

		//printf("In freshtimer,node time is %lu\r\n",nowtime);

		while(nowtime+10 > te->check_time){//处理超时的节点,应该是自己时间超过了对方检查的时间
			tenext=list_item_next(te);
			list_remove(wakeup_manager,te);
			//printf("L%u: node%u unDealNode\r\n",__LINE__,te->nodeId);
			TimeoutButNoDealNode(te);//处理这个超时的节点，并决定这个节点的去处

			te=tenext;
		}
		te=list_head(wakeup_manager);

		if(te){
			nowtime=call localtime.get();
			if(te->check_time <= nowtime) {printf("L%u: node%u fresh_checktimer\r\n",__LINE__,TOS_NODE_ID);fresh_checktimer();}//避免出现负数,如果出现负数，再次刷新定时器
			nextcktime=te->check_time-nowtime;
			call checkTimer.stop();
			call checkTimer.startOneShot(nextcktime);//提前5ms调用
			//printf("L%u: node%u Time: next check %u@%lu\r\n",__LINE__,TOS_NODE_ID,te->nodeId,te->check_time+((struct neig_tab_t*)te)->b+ADVANCE2CHECK);
		}else{//没有可以检查的节点了，怎么办，这时候最简单的办法
			//1.遍历邻居节点表，找到能用的节点
			//2.如果可用节点低于一定的阈值，节点就得唤醒了
			//printf("L%u: node%u fresh_check but no node\r\n",__LINE__,TOS_NODE_ID);
		}

	}
	//------------------------------------------------OK(未测)-------------------------------------------------
	/*
	 *注意这个算法并不是最终算法，只作为前期理论的验证，实际算法在 void obtainPotentialnode_real(void* payload)
	 */
	void obtainPotentialnode(void* payload,int32_t offsetTime){
		struct Neighbor* pset=(struct Neighbor*)payload; 
		//需要遍历三部分，wakeup_manager,service_manager,uncknode_manager,使用最粗暴的for循环遍历,抹掉所有已知的节点

		for(;pset->nodeId!=FREENODE;pset++){
			uint8_t target=pset->nodeId;
			struct sche_tab_t* te=list_head(wakeup_manager);
			if(target==TOS_NODE_ID) continue;//不能检查自己
			for(;te!=NULL;te=list_item_next(te)){
				if(te->nodeId==target){
					//printf("L%u: node%u is find in wakeup_manager\r\n",__LINE__,target);
					break;
				}             
			}
			if(te) continue;//在检查队列中那么此时不对这个节点做任何的改变

			te=list_head(service_manager);
			for(;te!=NULL;te=list_item_next(te)){
				if(te->nodeId==target){
                                       // printf("L%u: node%u is find in service_manager\r\n",__LINE__,target);
					//printf("Find target in service_manager\r\n");
					break;
				}             
			}
			if(te) continue;//在当前服务链表中也是如此

			te=list_head(uncknode_manager);//在服务链表的节点，如果等于目标值，就将引用计数加1，超过最大引用计数时，节点需要再次检查一次
			for(;te!=NULL;te=list_item_next(te)){//不用记录下一个，如果节点有节点早就退出了
				if(te->nodeId==target){

					struct cknd_tab_t* cte=(struct cknd_tab_t*)te;
                                        printf("L%u: node%u is find in uncknode_manager\r\n",__LINE__,target);
					//printf("Find target in uncknode_manager\r\n");
					cte->handle_times++; 
					if(cte->handle_times==CHECKUNKNOWNTIMES){
						cte->handle_times=0;
						cte->check_time=pset->offsetTime+offsetTime; 
						//从邻居节点中获取节点下次唤醒的时间,我擦，好像还得知道对方节点唤醒时间,偏移时间为接收到的时间减去包中时间，那么检查时间应该为+
						cte->type=UNKNOWNNODE;
						list_remove(uncknode_manager, cte);//从未检查链表中删除，添加到排队的链表中
						insertNode(wakeup_manager,(struct sche_tab_t*)cte);//在未检查链中移除节点，并将对方添加到检查链中
						printf("L%u: node%u UNKNOWN check by neigT@%lu，@my %lu\r\n",__LINE__,target,pset->offsetTime,te->check_time);
					}
					break;
				}           
			}
			if(te) continue;
			//最好不要直接插入到
                        printf("L%u: node%u is not find in manager\r\n",__LINE__,target);
			//printf("no find in manager\r\n");
			{
				//最后一种情况未知节点，图省事的话，直接将节点插入到wakeup_manager,实际会增加算法复杂度
				struct cknd_tab_t *cte=call CkndManager.insert(target);//创建目标节点存储空间
				if(cte==NULL){//移除未检查链表的链头，添加在链表尾巴，因为节点检查节点会放在链表头部
					cte=list_head(uncknode_manager); 
					call CkndManager.delete(cte->nodeId);   //这里只是抹除了节点编号而已，位置还可以直接用  
					list_remove(uncknode_manager,cte);//删除头节点  
				}
				if(cte==NULL){
					printf("L%u: node%u insert node in cknode_manager is error\r\n",__LINE__,target);return;
				}
				cte->nodeId=target;
				cte->type=UNKNOWNNODE;  
				cte->handle_times=0;
				cte->check_time=pset->offsetTime+offsetTime;
				insertNode(wakeup_manager,(struct sche_tab_t*)cte);//
				printf("L%u: node%u UNKNOWN new2check by neigT@%lu,@my %lu\r\n",__LINE__,target,pset->offsetTime,te->check_time);//不能检查自己！！
			}
		}

		fresh_checktimer();//最后刷新节点的唤醒时间，没有 节点很正常
		//printf("L%u: node%u fresh_checktimer\r\n",__LINE__,TOS_NODE_ID);
	}
	//------------------------------------------OK(未测)----------------------------------------------------
	/*
	 *在beacon中邻节点表中提取潜在的邻居节点，并将节点添加到检查队伍中去，如果队伍满，应该挤出检查次数最大的节点
	 *payload： 指的beacon中邻居节点集合
	 * 问题：是否需要提供好点的算法提高计算差集的速率呢？权宜之计，先用32字节实现完美散列吧（地址范围0-255）
	 */
	void obtainPotentialnode_real(void* payload){
		uint32_t st[8]={0};
		uint8_t tn,row,col;
		struct sche_tab_t* te;
		struct Neighbor* pset=(struct Neighbor*)payload;

		//两个目标：第一点确定没有的节点，第二点，
		//更新在检查的潜在节点，那先看潜在的节点，潜在的完了，去掉那些位，在检测邻节点表中的，随后剩下的就是新增检查的
		while(pset->nodeId!=FREENODE){
			tn=pset->nodeId;
			row=tn>>5; col=1<<(tn%32);
			st[row]|=col;
			pset++;
		}
		pset=(struct Neighbor*)payload;
		//遍历两遍链表吧
		te= list_head(wakeup_manager);
		for(;te!=NULL;te= list_item_next(te)){
			//检查的节点存在和潜在的节点这部分节点暂时，消除掉
			tn=te->nodeId;
			row=tn>>5;
			col=1<<(tn%32);

			if(st[row]&col) st[row]^=col; //清除节点 
		}

		//遍历一遍未检查的链表，更新引用计数，如果超过一定次数，将节点加入到的检查队列中
		te=list_head(uncknode_manager);
		for(;te!=NULL;te= list_item_next(te)){
			struct cknd_tab_t *cte=(struct cknd_tab_t*)te;
			tn=cte->nodeId;
			row=tn>>5;
			col=1<<(tn%32);

			if(st[row]&col){
				st[row]^=col; //清除节点
				cte->handle_times++; 
				if(cte->handle_times>CHECKUNKNOWNTIMES){
					pset=(struct Neighbor*)payload;
					while(pset->nodeId!=tn) pset++; //遍历次数较少
					cte->check_time=call localtime.get()+pset->offsetTime; //从邻居节点中获取节点下次唤醒的时间
					cte->type=UNKNOWNNODE;
					list_remove(uncknode_manager, cte);
					insertNode(wakeup_manager,(struct sche_tab_t*)cte);//在未检查链中移除节点，并将对方添加到检查链中
				}
			}    
		}
		//检查剩余的节点
		pset=(struct Neighbor*)payload;
		while(pset->nodeId!=FREENODE){
			struct cknd_tab_t *cte;
			tn=pset->nodeId;
			row=tn>>5;
			col=1<<(tn%32);

			if(st[row]&col){
				st[row]^=col;//1清除节点
				cte=call CkndManager.insert(tn);
				if(cte==NULL){//移除未检查链表的链头，添加在链表尾巴，因为节点检查节点会放在链表头部
					cte=list_head(uncknode_manager); 
					call CkndManager.delete(tn);   //这里只是抹除了节点编号而已，位置还可以直接用  
					list_remove(uncknode_manager,cte);//删除头节点  
				}
				list_add(uncknode_manager,cte);   //添加到尾部
				cte->nodeId=tn;
				cte->type=UNKNOWNNODE;  
				cte->handle_times=0;
			}
			pset++;
		}
		fresh_checktimer();
		//printf("L%u: node%u fresh_checktimer\r\n",__LINE__,TOS_NODE_ID);
	}
	/*
	 * 处理收到的ackbeacon不是自己的
	 */
	event void PWMacListener.messageFilter(message_t *msg,void *payload,uint8_t beaconType){

	}

	/*
	 *收到beacon的处理方法，这里是处理收到beacon的处理，没有收到在上面的AMsendDone中
	 * 附加收到ackBeacon的动作
	 *---------------------------注意要把service剩余的节点全部处理掉，因为这部分节点不会被服务！！--------------------------
	 */
	event void PWMacListener.messageSnoopFilter(message_t *msg,void *payload,uint8_t offset,uint8_t beaconType){

		//根据beaconType判断收到的beacon还是ackBeacon
		//如果ackBeacon 提取参数,添加节点到邻居节点表中，设添加到检查任务中
		cc2420_header_t *header=(cc2420_header_t *)call CC2420PacketBody.getHeader(msg);
		struct BaseBeaconP * base=(struct BaseBeaconP*)((uint8_t*)payload+offset);
		uint8_t srcId=header->src;//这样才对
		uint32_t currentTime=*((uint32_t*)payload);//只有第一个beacon这个时间才是有效的
		struct sche_tab_t *tenext,*te;
		struct neig_tab_t *neig;
		bool isInSet=0;
		bool insertbyack=0;

		static int32_t  node_offsetTime;
                if(!(srcId==6||srcId==8)) return;//避免其它实验的干扰
		//te=list_head(service_manager);
		neig=call NbrManager.find(srcId);
#if TIMESTAMP
   snoopFilterstart=call localtime.get();
#endif               
                 

		//测试协同机制------------------------------------
#if TESTCOORDINATION
		if(TOS_NODE_ID==8||TOS_NODE_ID==2){//这样就好像节点2事先知道节点6的存在，因为对于节点6，初始化beacon会收到，此后节点还是正常检查对方
		}else if(TOS_NODE_ID==6&&srcId==2){//6通过节点8发现节点2
			static uint16_t cnt=0;//beacon计数
			if(beaconType==INIT_BEACON) return;//屏蔽掉节点2所有的初始化beacon
			if(cnt==0) printf("To test unvalid Coordination....\r\n");
			cnt++;
			if(cnt<20) return;//屏蔽掉节点占空比beacon 刚才好像把srcId的部分beacon也屏蔽掉了
			if(cnt==20) printf("To test valid Coordination....\r\n");
		}else if(TOS_NODE_ID==6&&srcId!=8){//除了节点2，8 其他节点信息一律屏蔽
			//printf("L%u: node%u rece other node %u but undeal\r\n",__LINE__,TOS_NODE_ID,srcId);
			return;// 避免周围节点对这三个节点的干扰
		}
#endif               



//计算本地的偏移时间
               if(beaconType==INIT_BEACON||beaconType==BEACON){//莫名奇妙的收到的一个节点的邻居节点的时间错误的包
			node_offsetTime=rcvTime-currentTime;//只要节点有可能打时间我就计算一遍自身-邻节点时间
			//printf("L%u: node%u Time: currentTime:%lu,node_offsettime:%ld\r\n",__LINE__,srcId,currentTime,node_offsetTime);
                       /* if(beaconType==BEACON&&base->type==RENDEZ){//把参数打印出来
                        struct PseudoPara* neigInfo=&(((struct rendezBeaconP*)base) ->para);
                       // printf ("this time WakeupTime %lu\r\n",neigInfo->nextWakeupTime);
                      
                      }*/
		}

//判断节点是否重启：
               if(neig&&(beaconType==INIT_BEACON||beaconType==BEACON)){ 
			//先更新时钟漂移，再更新时钟偏差
                        int32_t _off_b=node_offsetTime-(-neig->b);//如果是正值，说明对方的时钟相对我是正偏,我勒个去正负号真***绕，这个b计算的位置应该是在什么时候
                        if(abs(_off_b - neig->offset_b)>30){//验证这个问题的有效性
                            //考虑把这个节点删除掉：两步：从三条链表中删除，将存储位置空出来
                            list_remove(uncknode_manager,neig);
                            list_remove(service_manager,neig);
                            list_remove(wakeup_manager,neig);
                            call NbrManager.delete(srcId);
                            neig=NULL;//最后一定不要忘了
                            printf("L%u: node%u is deleted #restart\r\n",__LINE__,srcId);
                        }
			//neig->b=-node_offsetTime;
			//printf("L%u: node%u Time:offset_b is %ld\r\n",__LINE__,neig->nodeId,neig->offset_b);
		}          

		// receBeacon=true;
		//先根据beacon类型，提取节点集合是否有我
		if(beaconType==INIT_BEACON){
			uint8_t index=0;
			uint8_t *pset=((struct InitBeaconP*)base) ->set;
#if DEBUG
			//printf("messageSnoopFilter has receive InitBeaconP\r\n");
			//打印节点的集合的前三个节点
			//   printf("InitBeacon has node:%u,%u,%u ...\r\n",pset[0],pset[1],pset[2]);
#endif
			while(index<INITSETSIZE&&pset[index]!=FREENODE){
#if DEBUG
				//      printf("part node %u: %u\r\n",index,pset[index]);//打印只是部分节点
#endif
				if(pset[index]==TOS_NODE_ID){
					isInSet=1;break;
				}
				index++;
			}

		}else if(beaconType==BEACON&&base->type==RENDEZ){
			uint8_t index=0;
			struct Neighbor *pset=((struct rendezBeaconP*)base)->set;
			//打印节点集合的前三个节点
			//printf("L%u: node%u Rendez rece:%u,%u,%u ...\r\n",__LINE__,srcId,pset[0].nodeId,pset[1].nodeId,pset[2].nodeId);
			while(index<INITSETSIZE&&pset[index].nodeId!=FREENODE){
				if(pset[index].nodeId==TOS_NODE_ID){
					isInSet=1;break;
				}
				index++;
			}
		}

		
#if TIMESTAMP
   snoopFilterl1=call localtime.get();
#endif
		/*
		 *处理节点上电重启的情况，比较典型的情况就是节点的当前时间，小于本次邻居节点检查时间,不过由于本次时间的计算的到了下一次，应该由节点自身的时间和推算出节点的当前的时间
		 */
		/*if(neig){
			int32_t num=node_offsetTime-neig->b;//注意有可能溢出，但溢出计算的值错误概率也极大
			num=num>0?num:-num;
			if(num>30){//超过30ms就认为节点已经重启,这样的话，节点基本上也不可用
                                printf("L%u: node%u was restart\r\n",__LINE__,neig->nodeId);
				call NbrManager.delete(neig->nodeId);
				neig=NULL;//这里一定要设置未空
			}
		}*/
		//如果没有节点，那么应尝试添加节点(注意新插入的节点这时应该进入检查链表中去
		if(neig==NULL){
			struct neig_tab_t *neig_te;
			if(beaconType==INIT_BEACON){
				//提取参数，开辟空间，设置节点类型KNOWNNODE，设置链路为单向还是双向

				neig_te=call NbrManager.neigInsert(srcId, &(((struct InitBeaconP*)base) ->para),rcvTime,currentTime); 
				//printf("L%u: node%u be inserted\r\n",__LINE__,srcId);         

				if(neig_te) {
					neig_te->link_status=isInSet?TWOWAY:ONEWAY;
					neig_te->type=KNOWNNODE;
				}
			}else if(beaconType==ACK_BEACON){
				//提取参数，开辟空间，设置节点类型KNOWNNODE

				ack_beacon_t* Ack=(ack_beacon_t*)base;
				uint8_t dest=Ack->dest;
				struct PseudoPara *neigInfo;
				uint32_t preBeaconTime;


				neigInfo=&(Ack->para);
				preBeaconTime=neigInfo->nextWakeupTime;

				neig_te=call NbrManager.neigInsert(header->src,neigInfo,preBeaconTime+node_offsetTime,preBeaconTime);
				insertbyack=1;
				printf("preBeacon time is %lu,node time is preBeaconTime+node_offsetTime %ld\r\n", preBeaconTime,preBeaconTime+node_offsetTime);

				
				printf("node  be inserted by ackBeacon in %u\r\n",__LINE__);
				if(neig_te) {
					//te->link_status=isInSet?TWOWAY:ONEWAY;
					neig_te->type=KNOWNNODE;
					//neig_te->check_time=calRendezvousTime(neig,0);//收到ackbeacon 立刻去听一次
					//insertNode(wakeup_manager,(struct sche_tab_t *)neig_te);
				} 
				//可通过ackBeacon中地址的或者和对方交换参数没有
				//没交换参数，交换参数，并设置为单向；//有，提取参数，位置为双向 
				if(dest==TOS_NODE_ID) neig_te->link_status=TWOWAY;
				else{
					neig_te->link_status=ONEWAY;
					generateNotifyData(node_offsetTime);
					call ListenBeacon.send(srcId,&NotifyData_msg,sizeof(struct NotifyData));
					printf("L%u: node%u a unknown ackBeacon,send NotifyData\r\n",__LINE__,srcId); 
				}  
			}else if(beaconType==BEACON){
				if(base->type==BASE){

					generateNotifyData(node_offsetTime);
					call ListenBeacon.send(srcId,&NotifyData_msg,sizeof(struct NotifyData));
					printf("L%u: node%u a unknown %u 'sbaseBeacon,send NotifyData\r\n",__LINE__,TOS_NODE_ID,srcId);
				}else if(base->type==RENDEZ){
					//提取参数，开辟空间，设置节点类型KNOWNNODE，如果集合有我设置为单向，没有单向，交换参数
					neig_te=call NbrManager.neigInsert(srcId, &(((struct  rendezBeaconP*)base) ->para),rcvTime,currentTime); 
					printf("L%u: node%u unknown %u is inserted\r\n",__LINE__,TOS_NODE_ID,srcId);
					if(neig_te) {
						neig_te->link_status=isInSet?TWOWAY:ONEWAY;
						neig_te->type=KNOWNNODE;
					}
					if(!isInSet){
						generateNotifyData(node_offsetTime);
						call ListenBeacon.send(srcId,&NotifyData_msg,sizeof(struct NotifyData));
						printf("L%u: node%u a unknown rendezBeacon,send NotifyData\r\n",__LINE__,srcId);
						//忙或不忙我只调动一次 
					}
				}
			}
			if(neig_te){
				list_remove(wakeup_manager, neig_te);//先移除重复地址的或者相同节点
				//计算节点下次唤醒时间
				printf("L%u: node%u calRendez at Inert new Node\r\n",__LINE__,srcId);
				neig_te->type=KNOWNNODE;
				//通过ack插入或者初始化beacon插入的节点要立刻检查

				neig_te->check_time=calRendezvousTime(neig_te,(insertbyack?0:1));
				insertNode(wakeup_manager,(struct sche_tab_t*)neig_te);
				// insertNode(service_manager,(struct sche_tab_t*)neig_te);//
				fresh_checktimer();//刷新检查定时器
				//printf("L%u: node%u fresh_checktimer\r\n",__LINE__,TOS_NODE_ID);
			}
		}else if(neig->link_status==ONEWAY){
			if(beaconType==INIT_BEACON){
				//单向还是双向
				neig->link_status=isInSet?TWOWAY:ONEWAY;
				//call NbrManager.updateNeigList(srcId, &(((struct InitBeaconP*)base) ->para),rcvTime,currentTime); //更新的时候，那么必须及时更新参数计算的过程

#if DEBUG
				if(neig->link_status==TWOWAY)
					printf("L%u: node%u change to TWOWAY by INIT_BEACON\r\n",__LINE__,neig->nodeId);
#endif
			}else if(beaconType==ACK_BEACON){
				//查看目的地址是谁，是我就是双向，不是就是单向
				ack_beacon_t* Ack=(ack_beacon_t*)base;
				uint8_t dest=Ack->dest;
				if(dest==TOS_NODE_ID) {
					neig->link_status=TWOWAY;
					printf("L%u: node%u change to TWOWAY by ACK_BEACON\r\n",__LINE__,neig->nodeId);
				}
				else{//一种是上次参数交互过程没有轮到我 或者两节点时单向，有陌生节点和对方发生参数交互，这是节点交换一次参数吧
					generateNotifyData(node_offsetTime);
					call ListenBeacon.send(neig->nodeId,&NotifyData_msg,sizeof(struct NotifyData));
					printf("L%u: node%u ONEWAY and ACKBEACON,send NotifyData\r\n",__LINE__,neig->nodeId);
				}
			}else if(beaconType==BEACON){
				//清空fail

				neig->fail_time=0;
				if(base->type==BASE){ 
					;//printf("L%u: node%u 's BaseBeacon\r\n",__LINE__,neig->nodeId);     
				}else if(base->type==RENDEZ){
					//查看集合是否有我，有我就是双向，没有就是单向
					
                                        //20170829 //添加检查时间的强制校准
                                       // struct PseudoPara* neigInfo=&(((struct rendezBeaconP*)base) ->para);
                                        //printf ("this time WakeupTime %lu\r\n",neigInfo->nextWakeupTime);
                                     //  printf("L%u: node%u 's RendezBeacon@%ld\r\n",__LINE__,neig->nodeId,neig->check_time+ADVANCE2CHECK-rcvTime);
                                    //    neig->check_time=rcvTime-ADVANCE2CHECK;//neigInfo->nextWakeupTime+9-neig->b;//使用未矫正的时钟偏移量
					neig->link_status=isInSet?TWOWAY:ONEWAY;
                                        
				}
			}

		}else if(neig->link_status==TWOWAY){
			if(beaconType==INIT_BEACON){
				//call NbrManager.updateNeigList(srcId, &(((struct InitBeaconP*)base) ->para),rcvTime,currentTime); 
				//重启暂不处理
			}else if(beaconType==ACK_BEACON){
				//不处理
				ack_beacon_t* Ack=(ack_beacon_t*)base;
				uint8_t dest=Ack->dest;
				if(dest==TOS_NODE_ID) printf("L%u: node%u 's ack for me TWOWAY\r\n",__LINE__,neig->nodeId); 
				else{printf("L%u: node%u 's ack for other TWOWAY\r\n",__LINE__,neig->nodeId);}
			}else if(beaconType==BEACON){
				//update nodeoffset

				//清空fail
				neig->fail_time=0;
				if(base->type==BASE){ 
					;//printf("L%u: node%u 's BaseBeacon\r\n",__LINE__,neig->nodeId);  

				}else if(base->type==RENDEZ){//直接使用预约点beacon更新参数得了！！！
					//查看集合是否有我，双向
					//没有设为单向
                                       //20170829 //添加检查时间的强制校准
                                      //  struct PseudoPara* neigInfo=&(((struct rendezBeaconP*)base) ->para);
                                        //printf ("this time WakeupTime %lu\r\n",neigInfo->nextWakeupTime);
                                      //  neig->check_time=rcvTime-ADVANCE2CHECK;//neigInfo->nextWakeupTime+9-neig->b;//使用current_time?
					neig->link_status=isInSet?TWOWAY:ONEWAY;
                                 //       printf("L%u: node%u 's RendezBeacon@%ld\r\n",__LINE__,neig->nodeId,neig->check_time+ADVANCE2CHECK-rcvTime);
				}
			}
		}
		//处理时钟偏移
		if(neig&&(beaconType==INIT_BEACON||beaconType==BEACON)){ 
			//先更新时钟漂移，再更新时钟偏差
			neig->offset_b=node_offsetTime-(-neig->b);//如果是正值，说明对方的时钟相对我是正偏,我勒个去正负号真***绕，这个b计算的位置应该是在什么时候
			neig->b=-node_offsetTime;
			//printf("L%u: node%u Time:offset_b is %ld\r\n",__LINE__,neig->nodeId,neig->offset_b);
		}
		// 初始化过程节点不进入的服务链
		if(call whetherInDC.isDC()==false){
			return;
		}

		
		if((beaconType==BEACON)&&(base->type==RENDEZ)){
#if TIMESTAMP
   obtainPstart=call localtime.get();
#endif                         
			obtainPotentialnode(((struct rendezBeaconP*)base) ->set,node_offsetTime);// 预约点的beacon的要提取的邻居节点集合
#if TIMESTAMP
   obtainPrend=call localtime.get();
#endif 
		}

		te=list_head(service_manager);
		/*if(te==NULL) printf("NULL node in service\r\n");*/
		// 占空比过程处理服务链，找到收到的节点
		for(;te!=NULL;te=tenext){//只处理服务链中第一个收到beacon的节点
			tenext=list_item_next(te);

			if(te->nodeId!=srcId){//先找到服务的节点
				/*list_remove(service_manager, te);//应该先移除
				  TimeoutButNoDealNode(te);*/
				continue;
			}
			//下面处理发送beacon信息的节点
                        //在这里矫正检查时间比较合适，不知道的节点，或者不是发往本节点的的beacon都会在这里被发现
                         if(beaconType==BEACON&&base->type==RENDEZ){
                               struct PseudoPara* neigInfo=&(((struct rendezBeaconP*)base) ->para);
                              neig->check_time=neigInfo->nextWakeupTime-neig->b-ADVANCE2CHECK;//只要是在服务节点，已知未知无所谓，并且收到是节点的beacon，这个时间就能矫正
                             // neig->check_time=rcvTime-ADVANCE2CHECK;//不能这样，因为对方现在也再算，你用这个时间相当与再次之后有对方了
                              
                         }
			if(te->type==KNOWNNODE){
				uint8_t ckinterval=0;
				if(beaconType==BEACON) {ckinterval=MAX_BEACONCOUNT-base->count+1;
					//printf("L%u: node%u cal by beacon RENDEZ? %u....\r\n",__LINE__,te->nodeId,base->type==RENDEZ);
				}
				else if(beaconType==INIT_BEACON) {ckinterval=3;//初始化节点每隔三个beacon就去检查一次
					//printf("L%u: node%u cal by InitBeacon...\r\n",__LINE__,te->nodeId);
				}

				list_remove(service_manager, te);//将te在链表中删除,即使需要交换参数，改变也是节点在链路状态，不会改变下次检查的时间
				//printf("L%u: node%u calRendez in Duty Listen undeal\r\n",__LINE__,te->nodeId);
				neig->check_time=calRendezvousTime(neig,ckinterval);//这里计算时间是不是会有重复计算，比如前面的节点刚刚插入的时刻
				insertNode(wakeup_manager,(struct sche_tab_t *)neig);

			}else if(te->type==UNKNOWNNODE){  //收到未知节点的basebeacon，而且还不知道参数，这种节点需要继续服务，因为节点此时会交换参数
				//这个节点需要继续服务：收到基本类型beacon，没有参数，现在需要交换参数后收到ackBeacon才决定，确定位置，没有到空包侦听那处理
				//如果知道参数的beacon中，无论单向还是双向都可以在下一个预约点检查,都检查参数
				printf("L%u: node%u UNKNOWN %u 's baseBeacon,so...\r\n",__LINE__,TOS_NODE_ID,te->nodeId);
			}

		}
		//20170815刷新定时器的任务应该放在最后
		//fresh_checktimer();//这里不应该刷新定时器，没什么不刷新，难道超时时一总刷新，应该是，因为节点此时还在服务超时的节点，刷新出新节点有点问题
#if TIMESTAMP
   snoopFilterend=call localtime.get();
#endif 
	}

	/*
	 * beacon发送是否成功，成功失败都不做任何处理
	 */
	event void beaconAppend.beaconAppendSendDone(message_t *msg,error_t error,uint8_t offset){
#if TIMESTAMP
     printf("TIMESTAMP:BeaconApstart,BeaconApend %lu %lu\r\n",BeaconApstart,BeaconApend);
#endif 
	}

	/*
	 *初始化超时动作,好像是没啥动作了
	 */
	event void whetherInDC.initDone(){
	}
	/*
	 * 在排序链表中插入一个节点
	 */
	void insertNode(list_t list,struct sche_tab_t *node){
		struct sche_tab_t *te=list_head(list);       
		if(!te) list_add(list,node);
		else{
			if(te->check_time > node->check_time) {
				list_push(list,node);
			}else{
				while(te->next!=NULL){
					if( ((struct sche_tab_t *)(te->next))->check_time > node->check_time){
						list_insert(list,te,node);
						break;
					}
					te=list_item_next(te);
				}
				if(te->next==NULL) te->next=(void*)node;
			}
		}
	}

	/*
	   list_init; list_head; list_add; list_push ;list_insert; list_item_next; list_remove;
	 */
	void list_init(list_t list){
		*list = NULL;
	}
	void *list_head(list_t list){
		return *list;
	}

	void *list_tail(list_t list)
	{
		struct list *l;

		if(*list == NULL) {
			return NULL;
		}

		for(l = *list; l->next != NULL; l = l->next);

		return l;
	}

	void list_add(list_t list, void *item){
		struct list *l;

		/* Make sure not to add the same element twice */
		list_remove(list, item);

		((struct list *)item)->next = NULL;

		l = list_tail(list);

		if(l == NULL) {
			*list = item;
		} else {
			l->next = item;
		}
	}

	void list_push(list_t list, void *item)
	{
		/*  struct list *l;*/

		/* Make sure not to add the same element twice */
		list_remove(list, item);

		((struct list *)item)->next = *list;
		*list = item;
	}

	void list_insert(list_t list, void *previtem, void *newitem){
		if(previtem == NULL) {
			list_push(list, newitem);
		} else {

			((struct list *)newitem)->next = ((struct list *)previtem)->next;
			((struct list *)previtem)->next = newitem;
		}
	}

	void *list_item_next(void *item){
		return item == NULL? NULL: ((struct list *)item)->next;
	}

	void list_remove(list_t list, void *item){
		struct list *l, *r;

		if(*list == NULL) {
			return;
		}

		r = NULL;
		for(l = *list; l != NULL; l = l->next) {
			if(l == item) {
				if(r == NULL) {
					/* First on list */
					*list = l->next;
				} else {
					/* Not first on list */
					r->next = l->next;
				}
				l->next = NULL;
				return;
			}
			r = l;
		}
	}


	/* 该函数未用！！！！！！！！
	 * 计算潜在节点的唤醒，指标好商量，算了统一一下吧：每听到beacon的次数超过一定次数就去检查
	 * 现在只是简单的处理一下检查次数的字段而已
	 */
	uint32_t calPotentialCktime(struct cknd_tab_t* nodeitem){
		nodeitem->handle_times++;
		if(nodeitem->handle_times==LISTENTIMES2CHECK){
			nodeitem->handle_times=0;
			//这个时间根本没法确定，所以的话，还是每次定时器唤醒后，处理一下吧
		}
		return nodeitem->check_time;
	}

	event void pseudoWakeupTimeCompute.pseudoWakeup(){};
	event void pseudoWakeupTimeCompute.advanceWakeup(){};
}
