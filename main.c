#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<assert.h>
#include<time.h>

#include "processor.h"
#include "configfile.h"
#include "memory_controller.h"
#include "scheduler.h"
#include "params.h"

#define MAXTRACELINESIZE 64
long long int BIGNUM = 1000000;

#define K 12
#define FRONT_WIDTH 15
#define HEIGHT 24
#define BLOCKS_NUM (1<<(HEIGHT))-1
#define LABEL_NUM 1<<(HEIGHT-1)
// #define NODE_NUM (1<<(HEIGHT))-1
#define PR_GROUP_SIZE 100
#define LATENCY_EFFICIENCY 0.6666
#define POWER_COMSUME 277.5
#define NEW_POWER_COMSUME 210
//extern int labels[BLOCKS_NUM]={0};
int previous_label=0;
int previous_label1=0;
int previous_label2=0;
int pos_map[BLOCKS_NUM]={0};
int inverse_pos[BLOCKS_NUM]={0};
int inverse_label[BLOCKS_NUM]={0};
int path[HEIGHT]={0};
int previous_path[HEIGHT]={0};
int previous_path1[HEIGHT]={0};
int previous_path2[HEIGHT]={0};
int data_empty[BLOCKS_NUM]={0};

double baseReadNum=0;//path_ORAM
double pathReadNum=0;//fork;+multi
	double d1_pathReadNum=0;
	double d2_pathReadNum=0;
	double d3_pathReadNum=0;
double PA_ReadNum=0;//PA;+multi
double path_PA_ReadNum=0;//PA+fork;+multi

double PAPR_ReadNum=0;//PA+PR;+multi
double PAPRA_ReadNum=0;//PA+PRA;+multi
double path_PAPR_ReadNum=0;//PA+PR+fork;+multi
double path_PAPRA_ReadNum=0;//PA+PRA+fork;+multi

double basePower=0;//path_ORAM
double pathPower=0;//fork;+multi
double PA_Power=0;//PA;+multi
double path_PA_Power=0;//PA+fork;+multi

double PAPR_Power=0;
double PAPRA_Power=0;
double path_PAPR_Power=0;
double path_PAPRA_Power=0;

int DTAG1=1;
int DTAG2=3;
int DTAG3=5;
int joint_path[LABEL_NUM]={0};
int isjoint[BLOCKS_NUM]={0};
int output_cnt=0;
int previous_addr=0;
int pre_joint_num=0;
int pre_read_num=HEIGHT;
int PR_group_cnt=1;
int PRA_group_cnt=1;
int path_PR_group_cnt=1;
int path_PRA_group_cnt=1;

void label2path(int label)
{
  int front=0;
  path[0]=0;
  for(int i=0;i<HEIGHT-1;i++)
  {
    if(label/(1<<(HEIGHT-i-2)))
      front=front*2+2;
    else front=front*2+1;
    label=label%(1<<(HEIGHT-i-2));
    path[i+1]=front;
  }
}

void pre_label2path(int previous_label)
{
  int front=0;
  previous_path[0]=0;
  for(int i=0;i<HEIGHT-1;i++)
  {
    if(previous_label/(1<<(HEIGHT-i-2)))
      front=front*2+2;
    else front=front*2+1;
    previous_label=previous_label%(1<<(HEIGHT-i-2));
    previous_path[i+1]=front;
  }
}

void pre_label12path(int previous_label1)
{
  int front=0;
  previous_path1[0]=0;
  int label_temp=previous_label1;
  for(int i=0;i<HEIGHT-1;i++)
  {
    if(label_temp/(1<<(HEIGHT-i-2)))
      front=front*2+2;
    else front=front*2+1;
    label_temp=label_temp%(1<<(HEIGHT-i-2));
    previous_path1[i+1]=front;
  }
}

void pre_label22path(int previous_label2)
{
  int front=0;
  previous_path2[0]=0;
  int label_temp=previous_label2;
  for(int i=0;i<HEIGHT-1;i++)
  {
    if(label_temp/(1<<(HEIGHT-i-2)))
      front=front*2+2;
    else front=front*2+1;
    label_temp=label_temp%(1<<(HEIGHT-i-2));
    previous_path2[i+1]=front;
  }
}

void pre_labelarray2path()
{
  // for(int k=0;k<2;k++)
  // {
  //   int front=0;
  //   int label_temp=previous_label_array[k];
  //   previous_path_array[k][0]=0;
  //   for(int i=0;i<HEIGHT-1;i++)
  //   {
  //     if(label_temp/(1<<(HEIGHT-i-2)))
  //       front=front*2+2;
  //     else front=front*2+1;
  //     label_temp=label_temp%(1<<(HEIGHT-i-2));
  //     previous_path_array[k][i+1]=front;
  //   }
  // }
  int front=0;
  int label_temp=previous_label1;
  previous_path1[0]=0;
  for(int i=0;i<HEIGHT-1;i++)
  {
    if(label_temp/(1<<(HEIGHT-i-2))==1)
      front=front*2+2;
    else front=front*2+1;
    label_temp=label_temp%(1<<(HEIGHT-i-2));
    previous_path1[i+1]=front;
  }
  // label_temp=previous_label_array[1];
  // previous_path_array[1][0]=0;
  // for(int i=0;i<HEIGHT-1;i++)
  // {
  //   if(label_temp/(1<<(HEIGHT-i-2)))
  //     front=front*2+2;
  //   else front=front*2+1;
  //   label_temp=label_temp%(1<<(HEIGHT-i-2));
  //   previous_path_array[1][i+1]=front;
  // }
}

void read_node(long long int physical_address)
{
  int np_addr=physical_address%BLOCKS_NUM;
  int this_label=inverse_label[np_addr];
  srand((unsigned)time(NULL));
  int new_pos_done=1;
  int next_label;
  int time_cnt=0;
  while(new_pos_done)
  {
    next_label=rand()%(LABEL_NUM);
    label2path(next_label);
    for(int i=HEIGHT-1;i>=0;i--)
    {
      if(data_empty[path[i]]==0)
      {
        data_empty[inverse_pos[np_addr]]=0;
        data_empty[path[i]]=1;
        pos_map[path[i]]=np_addr;
        inverse_label[np_addr]=next_label;
        inverse_pos[np_addr]=path[i];
        new_pos_done=0;
      }
    }
    // time_cnt++;
    // if(time_cnt>127) break;
  }
}

void read_node_PA(long long int physical_address)
{
  int np_addr=physical_address%BLOCKS_NUM;
  int this_label=inverse_label[np_addr];
  int this_pos=inverse_pos[np_addr];
  int row_num=np_addr/(1<<(HEIGHT-FRONT_WIDTH));
  srand((unsigned)time(NULL));
  int new_pos_done=1;
  int next_label;
  for(int i=0;i<(1<<(HEIGHT-FRONT_WIDTH));i++)
  {
    // int search_target=inverse_label[row_num*(1<<FRONT_WIDTH)+i];
    int search_target=inverse_label[row_num*(1<<(HEIGHT-FRONT_WIDTH))+i];
    // if(search_target!=0&&joint_path[search_target]<K&&row_num*(1<<FRONT_WIDTH)+i!=np_addr)
    if(joint_path[search_target]<K)
    {
      next_label=search_target;
      label2path(next_label);
      for(int i=HEIGHT-1;i>=0;i--)
      {
        if(data_empty[path[i]]==0)
        {
          data_empty[inverse_pos[np_addr]]=0;
          data_empty[path[i]]=1;
          pos_map[path[i]]=np_addr;
          inverse_label[np_addr]=next_label;
          inverse_pos[np_addr]=path[i];
          new_pos_done=0;
          if(i==HEIGHT-1)
            joint_path[next_label]++;
          else
          {
            int label_prefix=next_label/(1<<(HEIGHT-i-1));
            for(int j=0;j<(1<<(HEIGHT-i-1));j++)
            {
              if(joint_path[label_prefix*(1<<(HEIGHT-i-1))+j]<K)
              joint_path[label_prefix*(1<<(HEIGHT-i-1))+j]++;
            }
          }
          if(isjoint[np_addr]==1)
          {
            int this_level;
            for(this_level=1;this_level<=HEIGHT;this_level++)
            {
              if(this_pos<((1<<this_level)-1)) break;
            }
            int label_prefix=this_label/(1<<(HEIGHT-this_level));
            for(int j=0;j<(1<<(HEIGHT-this_level));j++)
            {
              if(joint_path[label_prefix*(1<<(HEIGHT-this_level))+j]>0) 
                joint_path[label_prefix*(1<<(HEIGHT-this_level))+j]--;
            }
          }
          // printf("count! ");
          isjoint[np_addr]=1;
          break;
        }
      }
    }
    if(new_pos_done==0) break;
  }
  if(new_pos_done==1)
  {
    while(new_pos_done)
    {
      next_label=rand()%(LABEL_NUM);
      label2path(next_label);
      for(int i=HEIGHT-1;i>=0;i--)
      {
        if(data_empty[path[i]]==0)
        {
          data_empty[inverse_pos[np_addr]]=0;
          data_empty[path[i]]=1;
          pos_map[path[i]]=np_addr;
          inverse_label[np_addr]=next_label;
          inverse_pos[np_addr]=path[i];
          new_pos_done=0;
          if(isjoint[np_addr]==1)
          {
            int this_level;
            for(this_level=1;this_level<=HEIGHT;this_level++)
            {
              if(this_pos<((1<<this_level)-1)) break;
            }
            int label_prefix=this_label/(1<<(HEIGHT-this_level));
            for(int j=0;j<(1<<(HEIGHT-this_level));j++)
            {
              if(joint_path[label_prefix*(1<<(HEIGHT-this_level))+j]>0) 
                joint_path[label_prefix*(1<<(HEIGHT-this_level))+j]--;
            }
          }
          isjoint[np_addr]=0;
          break;
        }
      }
    }
    // time_cnt++;
    // if(time_cnt>127) break;
  }
}

int get_read_num_oram_single(long long int physical_address)
{
  int read_num=HEIGHT;
  int np_addr=physical_address%BLOCKS_NUM;
  int this_label=inverse_label[np_addr];
  pre_label2path(previous_label);
  label2path(this_label);
  int joint_num=0;
  int cut_level=0;
  int pre_perfect_size=joint_path[previous_label];
  for(int i=1;i<HEIGHT;i++)
  {
    if(path[i]==previous_path[i])
    {
      read_num--;
      cut_level=i;
    }
  }
  if(cut_level==HEIGHT-1) joint_num=0;
  else
  {
    for(int i=cut_level+1;i<HEIGHT;i++)
    {
      int np=pos_map[path[i]];
      if(isjoint[np]==1)
      {
        for(int j=i;j<HEIGHT;j++)
        {
          if(np/(1<<(HEIGHT-FRONT_WIDTH))==pos_map[path[j]]/(1<<(HEIGHT-FRONT_WIDTH)))
          {
            joint_num++;
            break;
          }
        }
      }
    }
  }
  previous_label=this_label;
  // read_node(physical_address);
  read_node_PA(physical_address);
  output_cnt++;
  // if(output_cnt%1000==0)

  //printf("NO:%d ",output_cnt);//last hide here;

  // printf("done_well ");
  if(joint_num>=read_num) printf("warning!!\n");
  // printf("the isjoint array is:");
  // for(int i=0;i<BLOCKS_NUM;i++)
  //   printf("%d ", isjoint[i]);
  // printf("\n");
  // printf("the joint_path array is:");
  // for(int i=0;i<LABEL_NUM;i++)
  //   printf("%d ", joint_path[i]);
  // printf("\n");


  baseReadNum+=HEIGHT;
  basePower+=HEIGHT*POWER_COMSUME;
  pathReadNum+=read_num;
  pathPower+=read_num*POWER_COMSUME;
  srand((unsigned)time(NULL));
  int d_tag=rand()%(10);
  if(d_tag<1) d1_pathReadNum+=HEIGHT;
  else d1_pathReadNum+=read_num;
  if(d_tag<3) d2_pathReadNum+=HEIGHT;
  else d2_pathReadNum+=read_num;
  if(d_tag<5) d3_pathReadNum+=HEIGHT;
  else d3_pathReadNum+=read_num;
  // readTotalCnt+=read_num-joint_path[this_label]+joint_path[this_label]*0.718;
  PA_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  PA_Power+=(HEIGHT-joint_path[this_label])*POWER_COMSUME+joint_path[this_label]*NEW_POWER_COMSUME;
  path_PA_ReadNum+=read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  path_PA_Power+=(read_num-joint_num)*POWER_COMSUME+joint_num*NEW_POWER_COMSUME;
  // printf("read_num,joint_num is %d,%d\t",read_num,joint_num);

  if(PR_group_cnt!=1)
  {
  	if(isjoint[previous_addr])
  	{
  		if(joint_path[this_label]>=HEIGHT-pre_perfect_size)
  		{
  			PAPR_ReadNum+=HEIGHT*LATENCY_EFFICIENCY;
  			PAPR_Power+=HEIGHT*NEW_POWER_COMSUME;
  		}
  		else
  		{
  			PAPR_ReadNum+=HEIGHT-(pre_perfect_size+joint_path[this_label])+(pre_perfect_size+joint_path[this_label])*LATENCY_EFFICIENCY;
  			PAPR_Power+=(HEIGHT-pre_perfect_size-joint_path[this_label])*POWER_COMSUME+(pre_perfect_size+joint_path[this_label])*NEW_POWER_COMSUME;
  		}
  	}
  	else 
  	{
  		PAPR_ReadNum+=HEIGHT-pre_perfect_size+pre_perfect_size*LATENCY_EFFICIENCY;
  		PAPR_Power+=(HEIGHT-pre_perfect_size)*POWER_COMSUME+(pre_perfect_size*NEW_POWER_COMSUME);
  	}
  	if(PR_group_cnt==PR_GROUP_SIZE)
  	{
  		PAPR_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  		PAPR_Power+=(HEIGHT-joint_path[this_label])*POWER_COMSUME+(joint_path[this_label])*NEW_POWER_COMSUME;
  		PR_group_cnt=0;
  	}
  }

  if(path_PR_group_cnt!=1)
  {
  	if(isjoint[previous_addr])
  	{
  		if(joint_num>=pre_read_num-pre_joint_num) 
  			{
  				path_PAPR_ReadNum+=pre_read_num*LATENCY_EFFICIENCY;
  				path_PAPR_Power+=pre_read_num*NEW_POWER_COMSUME;
  			}
  		else 
  			{
  				path_PAPR_ReadNum+=pre_read_num-(pre_joint_num+joint_num)+(pre_joint_num+joint_num)*LATENCY_EFFICIENCY;
  				path_PAPR_Power+=(pre_read_num-(pre_joint_num+joint_num))*POWER_COMSUME+(pre_joint_num+joint_num)*NEW_POWER_COMSUME;
  			}
  	}
  	else 
  	{
  		path_PAPR_ReadNum+=pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY;
  		path_PAPR_Power+=(pre_read_num-pre_joint_num)*POWER_COMSUME+pre_joint_num*NEW_POWER_COMSUME;
  	}
  	if(path_PR_group_cnt==PR_GROUP_SIZE)
  	{
  		path_PAPR_ReadNum+=read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  		path_PAPR_Power+=(read_num-joint_num)*POWER_COMSUME+joint_num*NEW_POWER_COMSUME;
  		path_PR_group_cnt=0;
  	}
  }

  // path_PAPR_ReadNum+=pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY;
  // printf("pre_read_num,pre_joint_num is %d,%d\n", pre_read_num,pre_joint_num);
  // printf("%f,%f\n",read_num-joint_num+joint_num*LATENCY_EFFICIENCY,pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY);
  // printf("%f,%f,%f\n",path_PA_ReadNum,path_PAPR_ReadNum,path_PAPR_ReadNum-path_PA_ReadNum);

  if(PRA_group_cnt!=1)
  {
  	if(isjoint[previous_addr])
  	{
  		if(joint_path[this_label]>=HEIGHT-pre_perfect_size)
  		{
  			PAPRA_ReadNum+=HEIGHT*LATENCY_EFFICIENCY;
  			PAPRA_Power+=HEIGHT*NEW_POWER_COMSUME;
  		}
  		else
  		{
  			PAPRA_ReadNum+=HEIGHT-(pre_perfect_size+joint_path[this_label])+(pre_perfect_size+joint_path[this_label])*LATENCY_EFFICIENCY;
  			PAPRA_Power+=(HEIGHT-(pre_perfect_size+joint_path[this_label]))*POWER_COMSUME+(pre_perfect_size+joint_path[this_label])*NEW_POWER_COMSUME;
  		}
  	}
  	else if(pre_perfect_size>=joint_path[this_label])
  	{
  		PAPRA_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  		PAPRA_Power+=(HEIGHT-joint_path[this_label])*POWER_COMSUME+joint_path[this_label]*NEW_POWER_COMSUME;
  		PRA_group_cnt=0;
  	}
  	else
  	{
  		PAPRA_ReadNum+=HEIGHT-pre_perfect_size+pre_perfect_size*LATENCY_EFFICIENCY;
  		PAPRA_Power+=(HEIGHT-pre_perfect_size)*POWER_COMSUME+pre_perfect_size*NEW_POWER_COMSUME;
  	}
  	if(PRA_group_cnt==PR_GROUP_SIZE)
  	{
  		PAPRA_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  		PAPRA_Power+=(HEIGHT-joint_path[this_label])*POWER_COMSUME+joint_path[this_label]*NEW_POWER_COMSUME;
  		PRA_group_cnt=0;
  	}
  }

  if(path_PRA_group_cnt!=1)
  {
  	if(isjoint[previous_addr])
  	{
  		if(joint_num>=pre_read_num-pre_joint_num)
  		{
  			path_PAPRA_ReadNum+=pre_read_num*LATENCY_EFFICIENCY;
  			path_PAPRA_Power+=pre_read_num*NEW_POWER_COMSUME;
  		}
  		else
  		{
  			path_PAPRA_ReadNum+=pre_read_num-(pre_joint_num+joint_num)+(pre_joint_num+joint_num)*LATENCY_EFFICIENCY;
  			path_PAPRA_Power+=(pre_read_num-(pre_joint_num+joint_num))*POWER_COMSUME+(pre_joint_num+joint_num)*NEW_POWER_COMSUME;
  		}
  	}
  	else if(pre_joint_num>=joint_num)
  	{
  		path_PAPRA_ReadNum+=pre_read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  		path_PAPRA_Power+=(pre_read_num-joint_num)*POWER_COMSUME+joint_num*NEW_POWER_COMSUME;
  		path_PRA_group_cnt=0;
  	}
  	else
  	{
  		path_PAPRA_ReadNum+=pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY;
  		path_PAPRA_Power+=(pre_read_num-pre_joint_num)*POWER_COMSUME+pre_joint_num*NEW_POWER_COMSUME;
  	}
  	if(path_PRA_group_cnt==PR_GROUP_SIZE)
  	{
  		path_PAPRA_ReadNum+=read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  		path_PAPRA_Power+=(read_num-joint_num)*POWER_COMSUME+joint_num*NEW_POWER_COMSUME;
  		path_PRA_group_cnt=0;
  	}
  }

  // if(path_PRA_group_cnt!=1)
  // {
  // 	if(isjoint[previous_addr])
  // 	{
  // 		path_PAPRA_ReadNum+=pre_joint_num*LATENCY_EFFICIENCY;
  // 	}
  // 	else
  // 	{
  // 		path_PAPRA_ReadNum+=pre_read_num-pre_joint_num+joint_num*LATENCY_EFFICIENCY;
  // 		path_PRA_group_cnt=0;
  // 	}
  // 	if(PRA_group_cnt==PR_GROUP_SIZE)
  // 	{
  // 		path_PAPRA_ReadNum+=pre_joint_num*LATENCY_EFFICIENCY;
  // 		path_PAPRA_ReadNum+=read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  // 	}
  // }

  PR_group_cnt++;
  PRA_group_cnt++;
  path_PR_group_cnt++;
  path_PRA_group_cnt++;
  pre_joint_num=joint_num;
  pre_read_num=read_num;
  previous_addr=np_addr;
  // printf("done_well ");
  // return HEIGHT;
  return read_num;
}

int get_read_num_oram_multi(long long int physical_address)
{
  int read_num=HEIGHT;
  int np_addr=physical_address%BLOCKS_NUM;
  int this_label=inverse_label[np_addr];
  // pre_labelarray2path();
  pre_label12path(previous_label1);
  pre_label22path(previous_label2);
  label2path(this_label);
  int joint_num=0;
  int cut_level=0;
  int pre_perfect_size=joint_path[previous_label];
  for(int i=0;i<HEIGHT-1;i++)
  {
    if(path[i]==previous_path1[i]||path[i]==previous_path2[i])
    {
      read_num--;
      cut_level=i;
    }
  }
  if(cut_level==HEIGHT-1) joint_num=0;
  else
  {
    for(int i=cut_level+1;i<HEIGHT;i++)
    {
      int np=pos_map[path[i]];
      if(isjoint[np]==1)
      {
        for(int j=i;j<HEIGHT;j++)
        {
          if(np/(1<<(HEIGHT-FRONT_WIDTH))==pos_map[path[j]]/(1<<(HEIGHT-FRONT_WIDTH)))
          {
            joint_num++;
            break;
          }
        }
      }
    }
  }
  int p1p2=previous_label1^previous_label2;
  int tp1=previous_label1^this_label;
  int tp2=previous_label2^this_label;
  if(tp1>=p1p2&&tp1>=tp2)
    previous_label1=this_label;
  else if(tp2>=p1p2&&tp2>=tp1)
     previous_label2=this_label;
  // for(int i=1;i<HEIGHT;i++)
  // {
  //   int p1p2=previous_label_array[0]^previous_label_array[1];
  //   int tp1=previous_label_array[0]^this_label;
  //   int tp2=previous_label_array[1]^this_label;
  //   if(tp1>=p1p2&&tp1>=tp2)
  //     previous_label_array[1]=this_label;
  //   else if(tp2>=p1p2&&tp2>=tp1)
  //     previous_label_array[0]=this_label;
  // }
  // previous_label1=previous_label2;
  // previous_label2=this_label;
  // read_node(physical_address);
  read_node_PA(physical_address);
  output_cnt++;
  // if(output_cnt%1000==0)
 // printf("NO:%d ",output_cnt);
  // printf("done_well ");
  //printf("the isjoint array is:");
  //for(int i=0;i<BLOCKS_NUM;i++)
    //printf("%d ", isjoint[i]);
  //printf("\n");
  //printf("the joint_path array is:");
  //for(int i=0;i<LABEL_NUM;i++)
    //printf("%d ", joint_path[i]);
  //printf("\n");

  baseReadNum+=HEIGHT;
  basePower+=HEIGHT*POWER_COMSUME;
  pathReadNum+=read_num;
  pathPower+=read_num*POWER_COMSUME;
  srand((unsigned)time(NULL));
  int d_tag=rand()%(10);
  if(d_tag<1) d1_pathReadNum+=HEIGHT;
  else d1_pathReadNum+=read_num;
  if(d_tag<3) d2_pathReadNum+=HEIGHT;
  else d2_pathReadNum+=read_num;
  if(d_tag<5) d3_pathReadNum+=HEIGHT;
  else d3_pathReadNum+=read_num;
  // readTotalCnt+=read_num-joint_path[this_label]+joint_path[this_label]*0.718;
  PA_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  PA_Power+=(HEIGHT-joint_path[this_label])*POWER_COMSUME+joint_path[this_label]*NEW_POWER_COMSUME;
  path_PA_ReadNum+=read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  path_PA_Power+=(read_num-joint_num)*POWER_COMSUME+joint_num*NEW_POWER_COMSUME;
  // printf("read_num,joint_num is %d,%d\t",read_num,joint_num);

  if(PR_group_cnt!=1)
  {
  	if(isjoint[previous_addr])
  	{
  		if(joint_path[this_label]>=HEIGHT-pre_perfect_size)
  		{
  			PAPR_ReadNum+=HEIGHT*LATENCY_EFFICIENCY;
  			PAPR_Power+=HEIGHT*NEW_POWER_COMSUME;
  		}
  		else
  		{
  			PAPR_ReadNum+=HEIGHT-(pre_perfect_size+joint_path[this_label])+(pre_perfect_size+joint_path[this_label])*LATENCY_EFFICIENCY;
  			PAPR_Power+=(HEIGHT-pre_perfect_size-joint_path[this_label])*POWER_COMSUME+(pre_perfect_size+joint_path[this_label])*NEW_POWER_COMSUME;
  		}
  	}
  	else 
  	{
  		PAPR_ReadNum+=HEIGHT-pre_perfect_size+pre_perfect_size*LATENCY_EFFICIENCY;
  		PAPR_Power+=(HEIGHT-pre_perfect_size)*POWER_COMSUME+(pre_perfect_size*NEW_POWER_COMSUME);
  	}
  	if(PR_group_cnt==PR_GROUP_SIZE)
  	{
  		PAPR_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  		PAPR_Power+=(HEIGHT-joint_path[this_label])*POWER_COMSUME+(joint_path[this_label])*NEW_POWER_COMSUME;
  		PR_group_cnt=0;
  	}
  }

  if(path_PR_group_cnt!=1)
  {
  	if(isjoint[previous_addr])
  	{
  		if(joint_num>=pre_read_num-pre_joint_num) 
  			{
  				path_PAPR_ReadNum+=pre_read_num*LATENCY_EFFICIENCY;
  				path_PAPR_Power+=pre_read_num*NEW_POWER_COMSUME;
  			}
  		else 
  			{
  				path_PAPR_ReadNum+=pre_read_num-(pre_joint_num+joint_num)+(pre_joint_num+joint_num)*LATENCY_EFFICIENCY;
  				path_PAPR_Power+=(pre_read_num-(pre_joint_num+joint_num))*POWER_COMSUME+(pre_joint_num+joint_num)*NEW_POWER_COMSUME;
  			}
  	}
  	else 
  	{
  		path_PAPR_ReadNum+=pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY;
  		path_PAPR_Power+=(pre_read_num-pre_joint_num)*POWER_COMSUME+pre_joint_num*NEW_POWER_COMSUME;
  	}
  	if(path_PR_group_cnt==PR_GROUP_SIZE)
  	{
  		path_PAPR_ReadNum+=read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  		path_PAPR_Power+=(read_num-joint_num)*POWER_COMSUME+joint_num*NEW_POWER_COMSUME;
  		path_PR_group_cnt=0;
  	}
  }

  // path_PAPR_ReadNum+=pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY;
  // printf("pre_read_num,pre_joint_num is %d,%d\n", pre_read_num,pre_joint_num);
  // printf("%f,%f\n",read_num-joint_num+joint_num*LATENCY_EFFICIENCY,pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY);
  // printf("%f,%f,%f\n",path_PA_ReadNum,path_PAPR_ReadNum,path_PAPR_ReadNum-path_PA_ReadNum);

  if(PRA_group_cnt!=1)
  {
  	if(isjoint[previous_addr])
  	{
  		if(joint_path[this_label]>=HEIGHT-pre_perfect_size)
  		{
  			PAPRA_ReadNum+=HEIGHT*LATENCY_EFFICIENCY;
  			PAPRA_Power+=HEIGHT*NEW_POWER_COMSUME;
  		}
  		else
  		{
  			PAPRA_ReadNum+=HEIGHT-(pre_perfect_size+joint_path[this_label])+(pre_perfect_size+joint_path[this_label])*LATENCY_EFFICIENCY;
  			PAPRA_Power+=(HEIGHT-(pre_perfect_size+joint_path[this_label]))*POWER_COMSUME+(pre_perfect_size+joint_path[this_label])*NEW_POWER_COMSUME;
  		}
  	}
  	else if(pre_perfect_size>=joint_path[this_label])
  	{
  		PAPRA_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  		PAPRA_Power+=(HEIGHT-joint_path[this_label])*POWER_COMSUME+joint_path[this_label]*NEW_POWER_COMSUME;
  		PRA_group_cnt=0;
  	}
  	else
  	{
  		PAPRA_ReadNum+=HEIGHT-pre_perfect_size+pre_perfect_size*LATENCY_EFFICIENCY;
  		PAPRA_Power+=(HEIGHT-pre_perfect_size)*POWER_COMSUME+pre_perfect_size*NEW_POWER_COMSUME;
  	}
  	if(PRA_group_cnt==PR_GROUP_SIZE)
  	{
  		PAPRA_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  		PAPRA_Power+=(HEIGHT-joint_path[this_label])*POWER_COMSUME+joint_path[this_label]*NEW_POWER_COMSUME;
  		PRA_group_cnt=0;
  	}
  }

  if(path_PRA_group_cnt!=1)
  {
  	if(isjoint[previous_addr])
  	{
  		if(joint_num>=pre_read_num-pre_joint_num)
  		{
  			path_PAPRA_ReadNum+=pre_read_num*LATENCY_EFFICIENCY;
  			path_PAPRA_Power+=pre_read_num*NEW_POWER_COMSUME;
  		}
  		else
  		{
  			path_PAPRA_ReadNum+=pre_read_num-(pre_joint_num+joint_num)+(pre_joint_num+joint_num)*LATENCY_EFFICIENCY;
  			path_PAPRA_Power+=(pre_read_num-(pre_joint_num+joint_num))*POWER_COMSUME+(pre_joint_num+joint_num)*NEW_POWER_COMSUME;
  		}
  	}
  	else if(pre_joint_num>=joint_num)
  	{
  		path_PAPRA_ReadNum+=pre_read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  		path_PAPRA_Power+=(pre_read_num-joint_num)*POWER_COMSUME+joint_num*NEW_POWER_COMSUME;
  		path_PRA_group_cnt=0;
  	}
  	else
  	{
  		path_PAPRA_ReadNum+=pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY;
  		path_PAPRA_Power+=(pre_read_num-pre_joint_num)*POWER_COMSUME+pre_joint_num*NEW_POWER_COMSUME;
  	}
  	if(path_PRA_group_cnt==PR_GROUP_SIZE)
  	{
  		path_PAPRA_ReadNum+=read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  		path_PAPRA_Power+=(read_num-joint_num)*POWER_COMSUME+joint_num*NEW_POWER_COMSUME;
  		path_PRA_group_cnt=0;
  	}
  }

  // baseReadNum+=HEIGHT;
  // pathReadNum+=read_num;
  // // readTotalCnt+=read_num-joint_path[this_label]+joint_path[this_label]*0.718;
  // PA_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  // path_PA_ReadNum+=read_num-joint_num+joint_num*LATENCY_EFFICIENCY;

  // if(PR_group_cnt!=1)
  // {
  // 	if(isjoint[previous_addr])
  // 	{
  // 		if(joint_path[this_label]>=HEIGHT-pre_perfect_size)
  // 		{
  // 			// path_PAPR_ReadNum+=pre_read_num*LATENCY_EFFICIENCY;
  // 			PAPR_ReadNum+=HEIGHT*LATENCY_EFFICIENCY;
  // 		}
  // 		else
  // 		{
  // 			PAPR_ReadNum+=HEIGHT-(pre_perfect_size+joint_path[this_label])+(pre_perfect_size+joint_path[this_label])*LATENCY_EFFICIENCY;
  // 			// path_PAPR_ReadNum+=pre_read_num-(pre_joint_num+joint_num)+(pre_joint_num+joint_num)*LATENCY_EFFICIENCY;
  // 		}
  // 	}
  // 	else 
  // 	{
  // 		PAPR_ReadNum+=HEIGHT-pre_perfect_size+pre_perfect_size*LATENCY_EFFICIENCY;
  // 		// path_PAPR_ReadNum+=pre_read_num;
  // 	}
  // 	if(PR_group_cnt==PR_GROUP_SIZE)
  // 	{
  // 		PAPR_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  // 		// path_PAPR_ReadNum+=read_num;
  // 		PR_group_cnt=0;
  // 	}
  // }

  // if(path_PR_group_cnt!=1)
  // {
  // 	if(isjoint[previous_addr])
  // 	{
  // 		if(joint_num>=pre_read_num-pre_joint_num) path_PAPR_ReadNum+=pre_read_num*LATENCY_EFFICIENCY;
  // 		else path_PAPR_ReadNum+=pre_read_num-(pre_joint_num+joint_num)+(pre_joint_num+joint_num)*LATENCY_EFFICIENCY;
  // 	}
  // 	else path_PAPR_ReadNum+=pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY;
  // 	// path_PAPR_ReadNum+=pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY;
  // 	if(path_PR_group_cnt==PR_GROUP_SIZE)
  // 	{
  // 		path_PAPR_ReadNum+=read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  // 		path_PR_group_cnt=0;
  // 	}
  // }

  // // path_PAPR_ReadNum+=pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY;
  // // printf("pre_read_num,pre_joint_num is %d,%d\n", pre_read_num,pre_joint_num);
  // // printf("%f,%f\n",read_num-joint_num+joint_num*LATENCY_EFFICIENCY,pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY);
  // // printf("%f,%f,%f\n",path_PA_ReadNum,path_PAPR_ReadNum,path_PAPR_ReadNum-path_PA_ReadNum);

  // if(PRA_group_cnt!=1)
  // {
  // 	if(isjoint[previous_addr])
  // 	{
  // 		if(joint_path[this_label]>=HEIGHT-pre_perfect_size)
  // 			PAPRA_ReadNum+=HEIGHT*LATENCY_EFFICIENCY;
  // 		else
  // 			PAPRA_ReadNum+=HEIGHT-(pre_perfect_size+joint_path[this_label])+(pre_perfect_size+joint_path[this_label])*LATENCY_EFFICIENCY;
  // 	}
  // 	else if(pre_perfect_size>=joint_path[this_label])
  // 	{
  // 		PAPRA_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  // 		PRA_group_cnt=0;
  // 	}
  // 	else
  // 	{
  // 		PAPR_ReadNum+=HEIGHT-pre_perfect_size+pre_perfect_size*LATENCY_EFFICIENCY;
  // 	}
  // 	if(PRA_group_cnt==PR_GROUP_SIZE)
  // 	{
  // 		PAPRA_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  // 		PRA_group_cnt=0;
  // 	}
  // }

  // if(path_PRA_group_cnt!=1)
  // {
  // 	if(isjoint[previous_addr])
  // 	{
  // 		if(joint_num>=pre_read_num-pre_joint_num)
  // 			path_PAPRA_ReadNum+=pre_read_num*LATENCY_EFFICIENCY;
  // 		else
  // 			path_PAPRA_ReadNum+=pre_read_num-(pre_joint_num+joint_num)+(pre_joint_num+joint_num)*LATENCY_EFFICIENCY;
  // 	}
  // 	else if(pre_joint_num>=joint_num)
  // 	{
  // 		path_PAPRA_ReadNum+=pre_read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  // 		path_PRA_group_cnt=0;
  // 	}
  // 	else
  // 	{
  // 		path_PAPRA_ReadNum+=pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY;
  // 	}
  // 	if(path_PRA_group_cnt==PR_GROUP_SIZE)
  // 	{
  // 		path_PAPRA_ReadNum+=read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  // 		path_PRA_group_cnt=0;
  // 	}
  // }

  // if(path_PRA_group_cnt!=1)
  // {
  // 	if(isjoint[previous_addr])
  // 	{
  // 		path_PAPRA_ReadNum+=pre_joint_num*LATENCY_EFFICIENCY;
  // 	}
  // 	else
  // 	{
  // 		path_PAPRA_ReadNum+=pre_read_num-pre_joint_num+joint_num*LATENCY_EFFICIENCY;
  // 		path_PRA_group_cnt=0;
  // 	}
  // 	if(PRA_group_cnt==PR_GROUP_SIZE)
  // 	{
  // 		path_PAPRA_ReadNum+=pre_joint_num*LATENCY_EFFICIENCY;
  // 		path_PAPRA_ReadNum+=read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  // 	}
  // }

  PR_group_cnt++;
  PRA_group_cnt++;
  path_PR_group_cnt++;
  path_PRA_group_cnt++;
  pre_joint_num=joint_num;
  pre_read_num=read_num;
  previous_addr=np_addr;

  // printf("done_well ");
  // return HEIGHT;
  return read_num;
}

int get_read_num_oram_random_multi(long long int physical_address)
{
  int read_num=HEIGHT;
  int np_addr=physical_address%BLOCKS_NUM;
  int this_label=inverse_label[np_addr];
  // pre_labelarray2path();
  pre_label12path(previous_label1);
  pre_label22path(previous_label2);
  label2path(this_label);
  int joint_num=0;
  int cut_level=0;
  int pre_perfect_size=joint_path[previous_label];
  for(int i=0;i<HEIGHT-1;i++)
  {
    if(path[i]==previous_path1[i]||path[i]==previous_path2[i])
    {
      read_num--;
      cut_level=i;
    }
  }
  if(cut_level==HEIGHT-1) joint_num=0;
  else
  {
    for(int i=cut_level+1;i<HEIGHT;i++)
    {
      int np=pos_map[path[i]];
      if(isjoint[np]==1)
      {
        for(int j=i;j<HEIGHT;j++)
        {
          if(np/(1<<(HEIGHT-FRONT_WIDTH))==pos_map[path[j]]/(1<<(HEIGHT-FRONT_WIDTH)))
          {
            joint_num++;
            break;
          }
        }
      }
    }
  }
  int p1p2=previous_label1^previous_label2;
  int tp1=previous_label1^this_label;
  int tp2=previous_label2^this_label;
  if(tp1>=p1p2&&tp1>=tp2)
    previous_label2=this_label;
  else if(tp2>=p1p2&&tp2>=tp1)
     previous_label1=this_label;
  // for(int i=1;i<HEIGHT;i++)
  // {
  //   int p1p2=previous_label_array[0]^previous_label_array[1];
  //   int tp1=previous_label_array[0]^this_label;
  //   int tp2=previous_label_array[1]^this_label;
  //   if(tp1>=p1p2&&tp1>=tp2)
  //     previous_label_array[1]=this_label;
  //   else if(tp2>=p1p2&&tp2>=tp1)
  //     previous_label_array[0]=this_label;
  // }
  // previous_label1=previous_label2;
  // previous_label2=this_label;
  // read_node(physical_address);
  read_node_PA(physical_address);
  output_cnt++;
  // if(output_cnt%1000==0)
 // printf("NO:%d ",output_cnt);
  // printf("done_well ");
  //printf("the isjoint array is:");
  //for(int i=0;i<BLOCKS_NUM;i++)
    //printf("%d ", isjoint[i]);
  //printf("\n");
  //printf("the joint_path array is:");
  //for(int i=0;i<LABEL_NUM;i++)
    //printf("%d ", joint_path[i]);
  //printf("\n");

  baseReadNum+=HEIGHT;
  basePower+=HEIGHT*POWER_COMSUME;
  pathReadNum+=read_num;
  pathPower+=read_num*POWER_COMSUME;
  srand((unsigned)time(NULL));
  int d_tag=rand()%(10);
  if(d_tag<1) d1_pathReadNum+=HEIGHT;
  else d1_pathReadNum+=read_num;
  if(d_tag<3) d2_pathReadNum+=HEIGHT;
  else d2_pathReadNum+=read_num;
  if(d_tag<5) d3_pathReadNum+=HEIGHT;
  else d3_pathReadNum+=read_num;
  // readTotalCnt+=read_num-joint_path[this_label]+joint_path[this_label]*0.718;
  PA_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  PA_Power+=(HEIGHT-joint_path[this_label])*POWER_COMSUME+joint_path[this_label]*NEW_POWER_COMSUME;
  path_PA_ReadNum+=read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  path_PA_Power+=(read_num-joint_num)*POWER_COMSUME+joint_num*NEW_POWER_COMSUME;
  // printf("read_num,joint_num is %d,%d\t",read_num,joint_num);

  if(PR_group_cnt!=1)
  {
  	if(isjoint[previous_addr])
  	{
  		if(joint_path[this_label]>=HEIGHT-pre_perfect_size)
  		{
  			PAPR_ReadNum+=HEIGHT*LATENCY_EFFICIENCY;
  			PAPR_Power+=HEIGHT*NEW_POWER_COMSUME;
  		}
  		else
  		{
  			PAPR_ReadNum+=HEIGHT-(pre_perfect_size+joint_path[this_label])+(pre_perfect_size+joint_path[this_label])*LATENCY_EFFICIENCY;
  			PAPR_Power+=(HEIGHT-pre_perfect_size-joint_path[this_label])*POWER_COMSUME+(pre_perfect_size+joint_path[this_label])*NEW_POWER_COMSUME;
  		}
  	}
  	else 
  	{
  		PAPR_ReadNum+=HEIGHT-pre_perfect_size+pre_perfect_size*LATENCY_EFFICIENCY;
  		PAPR_Power+=(HEIGHT-pre_perfect_size)*POWER_COMSUME+(pre_perfect_size*NEW_POWER_COMSUME);
  	}
  	if(PR_group_cnt==PR_GROUP_SIZE)
  	{
  		PAPR_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  		PAPR_Power+=(HEIGHT-joint_path[this_label])*POWER_COMSUME+(joint_path[this_label])*NEW_POWER_COMSUME;
  		PR_group_cnt=0;
  	}
  }

  if(path_PR_group_cnt!=1)
  {
  	if(isjoint[previous_addr])
  	{
  		if(joint_num>=pre_read_num-pre_joint_num) 
  			{
  				path_PAPR_ReadNum+=pre_read_num*LATENCY_EFFICIENCY;
  				path_PAPR_Power+=pre_read_num*NEW_POWER_COMSUME;
  			}
  		else 
  			{
  				path_PAPR_ReadNum+=pre_read_num-(pre_joint_num+joint_num)+(pre_joint_num+joint_num)*LATENCY_EFFICIENCY;
  				path_PAPR_Power+=(pre_read_num-(pre_joint_num+joint_num))*POWER_COMSUME+(pre_joint_num+joint_num)*NEW_POWER_COMSUME;
  			}
  	}
  	else 
  	{
  		path_PAPR_ReadNum+=pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY;
  		path_PAPR_Power+=(pre_read_num-pre_joint_num)*POWER_COMSUME+pre_joint_num*NEW_POWER_COMSUME;
  	}
  	if(path_PR_group_cnt==PR_GROUP_SIZE)
  	{
  		path_PAPR_ReadNum+=read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  		path_PAPR_Power+=(read_num-joint_num)*POWER_COMSUME+joint_num*NEW_POWER_COMSUME;
  		path_PR_group_cnt=0;
  	}
  }

  // path_PAPR_ReadNum+=pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY;
  // printf("pre_read_num,pre_joint_num is %d,%d\n", pre_read_num,pre_joint_num);
  // printf("%f,%f\n",read_num-joint_num+joint_num*LATENCY_EFFICIENCY,pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY);
  // printf("%f,%f,%f\n",path_PA_ReadNum,path_PAPR_ReadNum,path_PAPR_ReadNum-path_PA_ReadNum);

  if(PRA_group_cnt!=1)
  {
  	if(isjoint[previous_addr])
  	{
  		if(joint_path[this_label]>=HEIGHT-pre_perfect_size)
  		{
  			PAPRA_ReadNum+=HEIGHT*LATENCY_EFFICIENCY;
  			PAPRA_Power+=HEIGHT*NEW_POWER_COMSUME;
  		}
  		else
  		{
  			PAPRA_ReadNum+=HEIGHT-(pre_perfect_size+joint_path[this_label])+(pre_perfect_size+joint_path[this_label])*LATENCY_EFFICIENCY;
  			PAPRA_Power+=(HEIGHT-(pre_perfect_size+joint_path[this_label]))*POWER_COMSUME+(pre_perfect_size+joint_path[this_label])*NEW_POWER_COMSUME;
  		}
  	}
  	else if(pre_perfect_size>=joint_path[this_label])
  	{
  		PAPRA_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  		PAPRA_Power+=(HEIGHT-joint_path[this_label])*POWER_COMSUME+joint_path[this_label]*NEW_POWER_COMSUME;
  		PRA_group_cnt=0;
  	}
  	else
  	{
  		PAPRA_ReadNum+=HEIGHT-pre_perfect_size+pre_perfect_size*LATENCY_EFFICIENCY;
  		PAPRA_Power+=(HEIGHT-pre_perfect_size)*POWER_COMSUME+pre_perfect_size*NEW_POWER_COMSUME;
  	}
  	if(PRA_group_cnt==PR_GROUP_SIZE)
  	{
  		PAPRA_ReadNum+=HEIGHT-joint_path[this_label]+joint_path[this_label]*LATENCY_EFFICIENCY;
  		PAPRA_Power+=(HEIGHT-joint_path[this_label])*POWER_COMSUME+joint_path[this_label]*NEW_POWER_COMSUME;
  		PRA_group_cnt=0;
  	}
  }

  if(path_PRA_group_cnt!=1)
  {
  	if(isjoint[previous_addr])
  	{
  		if(joint_num>=pre_read_num-pre_joint_num)
  		{
  			path_PAPRA_ReadNum+=pre_read_num*LATENCY_EFFICIENCY;
  			path_PAPRA_Power+=pre_read_num*NEW_POWER_COMSUME;
  		}
  		else
  		{
  			path_PAPRA_ReadNum+=pre_read_num-(pre_joint_num+joint_num)+(pre_joint_num+joint_num)*LATENCY_EFFICIENCY;
  			path_PAPRA_Power+=(pre_read_num-(pre_joint_num+joint_num))*POWER_COMSUME+(pre_joint_num+joint_num)*NEW_POWER_COMSUME;
  		}
  	}
  	else if(pre_joint_num>=joint_num)
  	{
  		path_PAPRA_ReadNum+=pre_read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  		path_PAPRA_Power+=(pre_read_num-joint_num)*POWER_COMSUME+joint_num*NEW_POWER_COMSUME;
  		path_PRA_group_cnt=0;
  	}
  	else
  	{
  		path_PAPRA_ReadNum+=pre_read_num-pre_joint_num+pre_joint_num*LATENCY_EFFICIENCY;
  		path_PAPRA_Power+=(pre_read_num-pre_joint_num)*POWER_COMSUME+pre_joint_num*NEW_POWER_COMSUME;
  	}
  	if(path_PRA_group_cnt==PR_GROUP_SIZE)
  	{
  		path_PAPRA_ReadNum+=read_num-joint_num+joint_num*LATENCY_EFFICIENCY;
  		path_PAPRA_Power+=(read_num-joint_num)*POWER_COMSUME+joint_num*NEW_POWER_COMSUME;
  		path_PRA_group_cnt=0;
  	}
  }

  PR_group_cnt++;
  PRA_group_cnt++;
  path_PR_group_cnt++;
  path_PRA_group_cnt++;
  pre_joint_num=joint_num;
  pre_read_num=read_num;
  previous_addr=np_addr;

  // printf("done_well ");
  // return HEIGHT;
  return read_num;
}




int expt_done=0;  

long long int CYCLE_VAL=0;

long long int get_current_cycle()
{
  return CYCLE_VAL;
}

struct robstructure *ROB;

FILE **tif;  /* The handles to the trace input files. */
FILE *config_file;
FILE *vi_file;

int *prefixtable;
// Moved the following to memory_controller.h so that they are visible
// from the scheduler.
//long long int *committed;
//long long int *fetched;
long long int *time_done;
long long int total_time_done;
float core_power=0;


int main(int argc, char * argv[])
{
  
  printf("---------------------------------------------\n");
  printf("-- USIMM: the Utah SImulated Memory Module --\n");
  printf("--              Version: 1.3               --\n");
  printf("---------------------------------------------\n");
  
  int numc=0;
  int num_ret=0;
  int num_fetch=0;
  int num_done=0;
  int numch=0;
  int writeqfull=0;
  int fnstart;
  int currMTapp;
  long long int maxtd;
  int maxcr;
  int pow_of_2_cores;
  char newstr[MAXTRACELINESIZE];
  int *nonmemops;
  char *opertype;
  long long int *addr;
  long long int *instrpc;
  int chips_per_rank=-1;

  /* Initialization code. */
  printf("Initializing.\n");

  if (argc < 3) {
    printf("Need at least one input configuration file and one trace file as argument.  Quitting.\n");
    return -3;
  }

  config_file = fopen(argv[1], "r");
  if (!config_file) {
    printf("Missing system configuration file.  Quitting. \n");
    return -4;
  }

  NUMCORES = argc-2;


  ROB = (struct robstructure *)malloc(sizeof(struct robstructure)*NUMCORES);
  tif = (FILE **)malloc(sizeof(FILE *)*NUMCORES);
  committed = (long long int *)malloc(sizeof(long long int)*NUMCORES);
  fetched = (long long int *)malloc(sizeof(long long int)*NUMCORES);
  time_done = (long long int *)malloc(sizeof(long long int)*NUMCORES);
  nonmemops = (int *)malloc(sizeof(int)*NUMCORES);
  opertype = (char *)malloc(sizeof(char)*NUMCORES);
  addr = (long long int *)malloc(sizeof(long long int)*NUMCORES);
  instrpc = (long long int *)malloc(sizeof(long long int)*NUMCORES);
  prefixtable = (int *)malloc(sizeof(int)*NUMCORES);
  currMTapp = -1;
  for (numc=0; numc < NUMCORES; numc++) {
     tif[numc] = fopen(argv[numc+2], "r");
     if (!tif[numc]) {
       printf("Missing input trace file %d.  Quitting. \n",numc);
       return -5;
     }

     /* The addresses in each trace are given a prefix that equals
        their core ID.  If the input trace starts with "MT", it is
	assumed to be part of a multi-threaded app.  The addresses
	from this trace file are given a prefix that equals that of
	the last seen input trace file that starts with "MT0".  For
	example, the following is an acceptable set of inputs for
	multi-threaded apps CG (4 threads) and LU (2 threads):
	usimm 1channel.cfg MT0CG MT1CG MT2CG MT3CG MT0LU MT1LU */
     prefixtable[numc] = numc;

     /* Find the start of the filename.  It's after the last "/". */
     for (fnstart = strlen(argv[numc+2]) ; fnstart >= 0; fnstart--) {
       if (argv[numc+2][fnstart] == '/') {
         break;
       }
     }
     fnstart++;  /* fnstart is either the letter after the last / or the 0th letter. */

     if ((strlen(argv[numc+2])-fnstart) > 2) {
       if ((argv[numc+2][fnstart+0] == 'M') && (argv[numc+2][fnstart+1] == 'T')) {
         if (argv[numc+2][fnstart+2] == '0') {
	   currMTapp = numc;
	 }
	 else {
	   if (currMTapp < 0) {
	     printf("Poor set of input parameters.  Input file %s starts with \"MT\", but there is no preceding input file starting with \"MT0\".  Quitting.\n", argv[numc+2]);
	     return -6;
	   }
	   else 
	     prefixtable[numc] = currMTapp;
	 }
       }
     }
     printf("Core %d: Input trace file %s : Addresses will have prefix %d\n", numc, argv[numc+2], prefixtable[numc]);

     committed[numc]=0;
     fetched[numc]=0;
     time_done[numc]=0;
     ROB[numc].head=0;
     ROB[numc].tail=0;
     ROB[numc].inflight=0;
     ROB[numc].tracedone=0;
  }

  read_config_file(config_file);


	/* Find the appropriate .vi file to read*/
	if (NUM_CHANNELS == 1 && NUMCORES == 1) {
  		vi_file = fopen("input/1Gb_x4.vi", "r"); 
		chips_per_rank= 16;
  		printf("Reading vi file: 1Gb_x4.vi\t\n%d Chips per Rank\n",chips_per_rank); 
	} else if (NUM_CHANNELS == 1 && NUMCORES == 2) {
  		vi_file = fopen("input/2Gb_x4.vi", "r");
		chips_per_rank= 16;
  		printf("Reading vi file: 2Gb_x4.vi\t\n%d Chips per Rank\n",chips_per_rank);
	} else if (NUM_CHANNELS == 1 && (NUMCORES > 2) && (NUMCORES <= 4)) {
  		vi_file = fopen("input/4Gb_x4.vi", "r");
		chips_per_rank= 16;
  		printf("Reading vi file: 4Gb_x4.vi\t\n%d Chips per Rank\n",chips_per_rank);
	} else if (NUM_CHANNELS == 4 && NUMCORES == 1) {
  		vi_file = fopen("input/1Gb_x16.vi", "r");
		chips_per_rank= 4;
  		printf("Reading vi file: 1Gb_x16.vi\t\n%d Chips per Rank\n",chips_per_rank);
	} else if (NUM_CHANNELS == 4 && NUMCORES == 2) {
  		vi_file = fopen("input/1Gb_x8.vi", "r");
		chips_per_rank= 8;
  		printf("Reading vi file: 1Gb_x8.vi\t\n%d Chips per Rank\n",chips_per_rank);
	} else if (NUM_CHANNELS == 4 && (NUMCORES > 2) && (NUMCORES <= 4)) {
  		vi_file = fopen("input/2Gb_x8.vi", "r");
		chips_per_rank= 8;
  		printf("Reading vi file: 2Gb_x8.vi\t\n%d Chips per Rank\n",chips_per_rank);
	} else if (NUM_CHANNELS == 4 && (NUMCORES > 4) && (NUMCORES <= 8)) {
  		vi_file = fopen("input/4Gb_x8.vi", "r");
		chips_per_rank= 8;
  		printf("Reading vi file: 4Gb_x8.vi\t\n%d Chips per Rank\n",chips_per_rank);
	} else if (NUM_CHANNELS == 4 && (NUMCORES > 8) && (NUMCORES <= 16)) {
  		vi_file = fopen("input/4Gb_x4.vi", "r");
		chips_per_rank= 16;
  		printf("Reading vi file: 4Gb_x4.vi\t\n%d Chips per Rank\n",chips_per_rank);
	} else {
		printf ("PANIC:: Channel - Core configuration not supported\n");
		assert (-1);
	}

  	if (!vi_file) {
 	  printf("Missing DRAM chip parameter file.  Quitting. \n");
  	  return -5;
  	}



  assert((log_base2(NUM_CHANNELS) + log_base2(NUM_RANKS) + log_base2(NUM_BANKS) + log_base2(NUM_ROWS) + log_base2(NUM_COLUMNS) + log_base2(CACHE_LINE_SIZE)) == ADDRESS_BITS );
  /* Increase the address space and rows per bank depending on the number of input traces. */
  ADDRESS_BITS = ADDRESS_BITS + log_base2(NUMCORES);
  if (NUMCORES == 1) {
    pow_of_2_cores = 1;
  }
  else {
  pow_of_2_cores = 1 << ((int)log_base2(NUMCORES-1) + 1);
  }
  NUM_ROWS = NUM_ROWS * pow_of_2_cores;

  read_config_file(vi_file);
  print_params();

  for(int i=0; i<NUMCORES; i++)
  {
	  ROB[i].comptime = (long long int*)malloc(sizeof(long long int)*ROBSIZE);
	  ROB[i].mem_address = (long long int*)malloc(sizeof(long long int)*ROBSIZE);
	  ROB[i].instrpc = (long long int*)malloc(sizeof(long long int)*ROBSIZE);
	  ROB[i].optype = (int*)malloc(sizeof(int)*ROBSIZE);
  }
  init_memory_controller_vars();
  init_scheduler_vars();
  /* Done initializing. */

  /* Must start by reading one line of each trace file. */
  for(numc=0; numc<NUMCORES; numc++)
  {
	      if (fgets(newstr,MAXTRACELINESIZE,tif[numc])) {
	        if (sscanf(newstr,"%d %c",&nonmemops[numc],&opertype[numc]) > 0) {
		  if (opertype[numc] == 'R') {
		    if (sscanf(newstr,"%d %c %Lx %Lx",&nonmemops[numc],&opertype[numc],&addr[numc],&instrpc[numc]) < 1) {
		      printf("Panic.  Poor trace format.\n");
		      return -4;
		    }
		  }
		  else {
		    if (opertype[numc] == 'W') {
		      if (sscanf(newstr,"%d %c %Lx",&nonmemops[numc],&opertype[numc],&addr[numc]) < 1) {
		        printf("Panic.  Poor trace format.\n");
		        return -3;
		      }
		    }
		    else {
		      printf("Panic.  Poor trace format.\n");
		      return -2;
		    }
		  }
		}
		else {
		  printf("Panic.  Poor trace format.\n");
		  return -1;
		}
	      }
	      else {
	        if (ROB[numc].inflight == 0) {
	          num_done++;
	          if (!time_done[numc]) time_done[numc] = 1;
	        }
	        ROB[numc].tracedone=1;
	      }
  }


  printf("Starting simulation.\n");
  while (!expt_done) {

    /* For each core, retire instructions if they have finished. */
    for (numc = 0; numc < NUMCORES; numc++) {
      num_ret = 0;
      while ((num_ret < MAX_RETIRE) && ROB[numc].inflight) {
        /* Keep retiring until retire width is consumed or ROB is empty. */
        if (ROB[numc].comptime[ROB[numc].head] < CYCLE_VAL) {  
	  /* Keep retiring instructions if they are done. */
	  ROB[numc].head = (ROB[numc].head + 1) % ROBSIZE;
	  ROB[numc].inflight--;
	  committed[numc]++;
	  num_ret++;
        }
	else  /* Instruction not complete.  Stop retirement for this core. */
	  break;
      }  /* End of while loop that is retiring instruction for one core. */
    }  /* End of for loop that is retiring instructions for all cores. */


    if(CYCLE_VAL%PROCESSOR_CLK_MULTIPLIER == 0)
    { 
      /* Execute function to find ready instructions. */
      update_memory();

      /* Execute user-provided function to select ready instructions for issue. */
      /* Based on this selection, update DRAM data structures and set 
	 instruction completion times. */
      for(int c=0; c < NUM_CHANNELS; c++)
      {
	schedule(c);
	gather_stats(c);	
      }
    }

    /* For each core, bring in new instructions from the trace file to
       fill up the ROB. */
    num_done = 0;
    writeqfull =0;
    for(int c=0; c<NUM_CHANNELS; c++){
	    if(write_queue_length[c] == WQ_CAPACITY)
	    {
		    writeqfull = 1;
		    break;
	    }
    }

    for (numc = 0; numc < NUMCORES; numc++) {
      if (!ROB[numc].tracedone) { /* Try to fetch if EOF has not been encountered. */
        num_fetch = 0;
        while ((num_fetch < MAX_FETCH) && (ROB[numc].inflight != ROBSIZE) && (!writeqfull)) {
          /* Keep fetching until fetch width or ROB capacity or WriteQ are fully consumed. */
	  /* Read the corresponding trace file and populate the tail of the ROB data structure. */
	  /* If Memop, then populate read/write queue.  Set up completion time. */

	  if (nonmemops[numc]) {  /* Have some non-memory-ops to consume. */
	    ROB[numc].optype[ROB[numc].tail] = 'N';
	    ROB[numc].comptime[ROB[numc].tail] = CYCLE_VAL+PIPELINEDEPTH;
	    nonmemops[numc]--;
	    ROB[numc].tail = (ROB[numc].tail +1) % ROBSIZE;
	    ROB[numc].inflight++;
	    fetched[numc]++;
	    num_fetch++;
	  }
	  else { /* Done consuming non-memory-ops.  Must now consume the memory rd or wr. */
	      if (opertype[numc] == 'R') {
		  addr[numc] = addr[numc] + (long long int)((long long int)prefixtable[numc] << (ADDRESS_BITS - log_base2(NUMCORES)));    // Add MSB bits so each trace accesses a different address space.
	          ROB[numc].mem_address[ROB[numc].tail] = addr[numc];
	          ROB[numc].optype[ROB[numc].tail] = opertype[numc];
	          ROB[numc].comptime[ROB[numc].tail] = CYCLE_VAL + BIGNUM;
	          ROB[numc].instrpc[ROB[numc].tail] = instrpc[numc];
		
		  // Check to see if the read is for buffered data in write queue - 
		  // return constant latency if match in WQ
		  // add in read queue otherwise
		  int lat = read_matches_write_or_read_queue(addr[numc]);
		  if(lat) {
			ROB[numc].comptime[ROB[numc].tail] = CYCLE_VAL+lat+PIPELINEDEPTH;
		  }
		  else {
        // int redundant=cal_redundant(addr[numc]);
        int read_num=get_read_num_oram_random_multi(addr[numc]);
        if(read_num!=0)
        for(int recycle_time=0;recycle_time<read_num;recycle_time++)
			     insert_read(addr[numc], CYCLE_VAL, numc, ROB[numc].tail, instrpc[numc]);
		  }
	      }
	      else {  /* This must be a 'W'.  We are confirming that while reading the trace. */
	        if (opertype[numc] == 'W') {
		      addr[numc] = addr[numc] + (long long int)((long long int)prefixtable[numc] << (ADDRESS_BITS - log_base2(NUMCORES)));    // Add MSB bits so each trace accesses a different address space.
		      ROB[numc].mem_address[ROB[numc].tail] = addr[numc];
		      ROB[numc].optype[ROB[numc].tail] = opertype[numc];
		      ROB[numc].comptime[ROB[numc].tail] = CYCLE_VAL+PIPELINEDEPTH;
		      /* Also, add this to the write queue. */

		      if(!write_exists_in_write_queue(addr[numc]))
			insert_write(addr[numc], CYCLE_VAL, numc, ROB[numc].tail);

		      for(int c=0; c<NUM_CHANNELS; c++){
			if(write_queue_length[c] == WQ_CAPACITY)
			{
			  writeqfull = 1;
			  break;
			}
		      }
		}
		else {
		  printf("Panic.  Poor trace format. \n");
		  return -1;
		}
	      }
	      ROB[numc].tail = (ROB[numc].tail +1) % ROBSIZE;
	      ROB[numc].inflight++;
	      fetched[numc]++;
	      num_fetch++;

	      /* Done consuming one line of the trace file.  Read in the next. */
	      if (fgets(newstr,MAXTRACELINESIZE,tif[numc])) {
	        if (sscanf(newstr,"%d %c",&nonmemops[numc],&opertype[numc]) > 0) {
		  if (opertype[numc] == 'R') {
		    if (sscanf(newstr,"%d %c %Lx %Lx",&nonmemops[numc],&opertype[numc],&addr[numc],&instrpc[numc]) < 1) {
		      printf("Panic.  Poor trace format.\n");
		      return -4;
		    }
		  }
		  else {
		    if (opertype[numc] == 'W') {
		      if (sscanf(newstr,"%d %c %Lx",&nonmemops[numc],&opertype[numc],&addr[numc]) < 1) {
		        printf("Panic.  Poor trace format.\n");
		        return -3;
		      }
		    }
		    else {
		      printf("Panic.  Poor trace format.\n");
		      return -2;
		    }
		  }
		}
		else {
		  printf("Panic.  Poor trace format.\n");
		  return -1;
		}
	      }
	      else {
	        if (ROB[numc].inflight == 0) {
	          num_done++;
	          if (!time_done[numc]) time_done[numc] = CYCLE_VAL;
	        }
	        ROB[numc].tracedone=1;
	        break;  /* Break out of the while loop fetching instructions. */
	      }
	      
	  }  /* Done consuming the next rd or wr. */

	} /* One iteration of the fetch while loop done. */
      } /* Closing brace for if(trace not done). */
      else { /* Input trace is done.  Check to see if all inflight instrs have finished. */
        if (ROB[numc].inflight == 0) {
	  num_done++;
	  if (!time_done[numc]) time_done[numc] = CYCLE_VAL;
	}
      }
    } /* End of for loop that goes through all cores. */


    if (num_done == NUMCORES) {
      /* Traces have been consumed and in-flight windows are empty.  Must confirm that write queues have been drained. */
      for (numch=0;numch<NUM_CHANNELS;numch++) {
        if (write_queue_length[numch]) break;
      }
      if (numch == NUM_CHANNELS) expt_done=1;  /* All traces have been consumed and the write queues are drained. */
    }

    /* Printing details for testing.  Remove later. */
    //printf("Cycle: %lld\n", CYCLE_VAL);
    //for (numc=0; numc < NUMCORES; numc++) {
     // printf("C%d: Inf %d : Hd %d : Tl %d : Comp %lld : type %c : addr %x : TD %d\n", numc, ROB[numc].inflight, ROB[numc].head, ROB[numc].tail, ROB[numc].comptime[ROB[numc].head], ROB[numc].optype[ROB[numc].head], ROB[numc].mem_address[ROB[numc].head], ROB[numc].tracedone);
    //}

    CYCLE_VAL++;  /* Advance the simulation cycle. */
  }


  /* Code to make sure that the write queue drain time is included in
     the execution time of the thread that finishes last. */
  maxtd = time_done[0];
  maxcr = 0;
  for (numc=1; numc < NUMCORES; numc++) {
    if (time_done[numc] > maxtd) {
      maxtd = time_done[numc];
      maxcr = numc;
    }
  }
  time_done[maxcr] = CYCLE_VAL;

  core_power = 0;
  for (numc=0; numc < NUMCORES; numc++) {
    /* A core has peak power of 10 W in a 4-channel config.  Peak power is consumed while the thread is running, else the core is perfectly power gated. */
    core_power = core_power + (10*((float)time_done[numc]/(float)CYCLE_VAL));
  }
  if (NUM_CHANNELS == 1) {
    /* The core is more energy-efficient in our single-channel configuration. */
    core_power = core_power/2.0 ;
  }



  printf("Done with loop. Printing stats.\n");
  printf("Cycles %lld\n", CYCLE_VAL);
  total_time_done = 0;
  for (numc=0; numc < NUMCORES; numc++) {
    printf("Done: Core %d: Fetched %lld : Committed %lld : At time : %lld\n", numc, fetched[numc], committed[numc], time_done[numc]);
    total_time_done += time_done[numc];
  }
  printf("Sum of execution times for all programs: %lld\n", total_time_done);
  printf("Num reads merged: %lld\n",num_read_merge);
  printf("Num writes merged: %lld\n",num_write_merge);
  /* Print all other memory system stats. */
  scheduler_stats();
  print_stats();  

  /*Print Cycle Stats*/
  for(int c=0; c<NUM_CHANNELS; c++)
	  for(int r=0; r<NUM_RANKS ;r++)
		  calculate_power(c,r,0,chips_per_rank);

	printf ("\n#-------------------------------------- Power Stats ----------------------------------------------\n");
	printf ("Note:  1. termRoth/termWoth is the power dissipated in the ODT resistors when Read/Writes terminate \n");
	printf ("          in other ranks on the same channel\n");
	printf ("#-------------------------------------------------------------------------------------------------\n\n");


  /*Print Power Stats*/
	float total_system_power =0;
  for(int c=0; c<NUM_CHANNELS; c++)
	  for(int r=0; r<NUM_RANKS ;r++)
		  total_system_power += calculate_power(c,r,1,chips_per_rank);

		printf ("\n#-------------------------------------------------------------------------------------------------\n");
	if (NUM_CHANNELS == 4) {  /* Assuming that this is 4channel.cfg  */
	  printf ("Total memory system power = %f W\n",total_system_power/1000);
	  printf("Miscellaneous system power = 40 W  # Processor uncore power, disk, I/O, cooling, etc.\n");
	  printf("Processor core power = %f W  # Assuming that each core consumes 10 W when running\n",core_power);
	  printf("Total system power = %f W # Sum of the previous three lines\n", 40 + core_power + total_system_power/1000);
	  printf("Energy Delay product (EDP) = %2.9f J.s\n", (40 + core_power + total_system_power/1000)*(float)((double)CYCLE_VAL/(double)3200000000) * (float)((double)CYCLE_VAL/(double)3200000000));
	}
	else {  /* Assuming that this is 1channel.cfg  */
	  printf ("Total memory system power = %f W\n",total_system_power/1000);
	  printf("Miscellaneous system power = 10 W  # Processor uncore power, disk, I/O, cooling, etc.\n");  /* The total 40 W misc power will be split across 4 channels, only 1 of which is being considered in the 1-channel experiment. */
	  printf("Processor core power = %f W  # Assuming that each core consumes 5 W\n",core_power);  /* Assuming that the cores are more lightweight. */
	  printf("Total system power = %f W # Sum of the previous three lines\n", 10 + core_power + total_system_power/1000);
	  printf("Energy Delay product (EDP) = %2.9f J.s\n", (10 + core_power + total_system_power/1000)*(float)((double)CYCLE_VAL/(double)3200000000) * (float)((double)CYCLE_VAL/(double)3200000000));
	}

  return 0;
}







