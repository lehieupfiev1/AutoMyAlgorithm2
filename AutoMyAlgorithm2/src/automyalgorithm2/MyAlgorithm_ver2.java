/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package automyalgorithm2;

import static automyalgorithm2.SensorUtility.*;
import ilog.concert.IloException;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Hieu
 */
public class MyAlgorithm_ver2 {
public float Distance[][];// Matrix distance between two nodes
    public float MinDistanceSink[];// Matrix distance between two nodes
    public float Target[][];// Target nodes
    public float Point[][];// Total nodes
    public float Sink[][];// Target covering sensors
    
    double mTimeLife;
    float Rs, Rc;// Rs and Rt value
    float R; // Tinh phan chia nho nhat
    int MaxHopper;

    List<List<List<PathItem>>> SaveListofListY;
    List<List<List<Double>>> SaveListofListTi;
    List<List<Integer>> SaveListTarget;
    List<List<Integer>> SaveListSensor;
    List<List<Integer>> SaveListSink;
    List<List<PathItem>> resultListY;
    List<List<Double>> resultListTi;
    float ListEnergySensor[];
    float ListEnergyUsing[];
    int MAX_INTERGER = 100000000;
    float MAX_FLOAT = 10000000000000.0f;
    float TimeStamp ;
    boolean isWidthOptimal = false;
    boolean isHeightOptimal = false;
    
    float Es, Et,Er,Efs,Emp,Do, bit;
    int cnt;
    
    int K;// Number Sink
    int N;//Number sensor
    int TP; // Total points (Contain Sensor , Sink, Target )
    int T;//Number of Tagert Nodes
    int Anpha; // So lan tang
    
    public MyAlgorithm_ver2() {
        
    }
    
    public void run() {
        
        init();

        readData();
        //Step 1: Find target-covering Sensor
        //FindTargetCoveringSensor();
        
        //Step 2: 

        runAlgorithm();
        
        CoppyToListSensor();
//        
        freeData();
//        
        System.gc();
    }
    
    public void init() {
        resultListY = new ArrayList<>();
        resultListTi = new ArrayList<>();
        SaveListofListY = new ArrayList<>();
        SaveListofListTi = new ArrayList<>();
        SaveListTarget = new ArrayList<>();
        SaveListSink = new ArrayList<>();
    }
    
    public  void readData() {
        // Read Rs, Rc
        Rs = SensorUtility.mRsValue;
        Rc = SensorUtility.mRcValue;
        mTimeLife = 0;
        MaxHopper = SensorUtility.mMaxHopper;
        R = Rs + MaxHopper*Rc;
        
        //Read constance Energy : Es, Et,Er,Efs,Emp
        Es = SensorUtility.mEsValue;
        Et = SensorUtility.mEtValue;
        Er = SensorUtility.mErValue;
        Efs = SensorUtility.mEfsValue;
        Emp = SensorUtility.mEmpValue;
        Do = (float)Math.sqrt(Efs/Emp);
        bit = SensorUtility.mBitValue;
        TimeStamp = SensorUtility.mTstamp;
        Anpha = SensorUtility.Lvalue;
                
        //Read Sensor , Sink, Target 
        N = SensorUtility.mListSensorNodes.size();
        T = SensorUtility.mListTargetNodes.size();
        K = SensorUtility.mListSinkNodes.size();
        TP = N+ T+ K;
        
        //Add to Total Point;
        Point = new float[TP+1][2];
        ListEnergySensor = new float[N];
        ListEnergyUsing = new float[N];

        for (int i =0; i < mListSensorNodes.size();i++) {
            Point[i][0] = mListSensorNodes.get(i).getX();
            Point[i][1] = mListSensorNodes.get(i).getY();
            //Add Energy for every node
            ListEnergySensor[i] = SensorUtility.mEoValue;
            ListEnergyUsing[i] = 0;
        }
        
        for (int i =0; i < mListTargetNodes.size();i++) {
            Point[N+i][0] = mListTargetNodes.get(i).getX();
            Point[N+i][1] = mListTargetNodes.get(i).getY();
        }
        for (int i =0; i < mListSinkNodes.size();i++) {
            Point[N+T+i][0] = mListSinkNodes.get(i).getX();
            Point[N+T+i][1] = mListSinkNodes.get(i).getY();
        }

        // Create matrix distance
        Distance = new float[TP+1][TP+1];
         for (int i =0;i<TP;i++) {
            for (int j =0;j<=i;j++) {
                if (i==j ) {
                    Distance[i][j] = 0;
                } else {
                    Distance[i][j] = Distance[j][i] = calculateDistance(Point[i][0], Point[i][1], Point[j][0], Point[j][1]);
                }
            }
        }
         
        //Caculate Mindistance form sensor to Sink
        MinDistanceSink = new float[N];
        float min;
        for (int i =0; i<N ;i++) {
            min = MAX_FLOAT;
            for (int j =0; j < K; j++) {
                if (Distance[i][N+T+j] < min) {
                    min = Distance[i][N+T+j];
                }
            }
            MinDistanceSink[i] = min;
        }
        
        //Init resultListY and resultListTi 
        
        for (int i = 0 ;i < T; i++) {
            List<PathItem> pathY = new ArrayList<>();
            List<Double> timeY = new ArrayList<>();
            resultListY.add(pathY);
            resultListTi.add(timeY);
        }

    }
    
    public void freeData() {
        MinDistanceSink = null;
        SaveListofListY = null;
        SaveListofListTi = null;
        SaveListTarget = null;
        SaveListSensor = null;
        Point = null;
        Distance = null;
        SaveListSink = null;
        ListEnergySensor = null;
    }
    
    public  float calculateDistance(float x1, float y1, float x2, float y2) {
        return (float) Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }
    
    boolean checkSensorConnectSink(int sensor, List<Integer> listSink) {
        for (int i =0; i< listSink.size();i++) {
            if (Distance[sensor][N+T+listSink.get(i)] <= Rc) {
                return true;
            }
        }
        return false;
    }
    
    void Finding_CCP2(List<Integer> listSensor, List<Integer> listTarget, List<Integer> listSink, List<List<PathItem>> ListPathY) {
        List<List<List<Integer>>> ListPi = new ArrayList<>();
        List<List<Integer>> ListP = new ArrayList<>();
        List<List<Integer>> ListParent = new ArrayList<>();
        
        //Check nulll
        if (listSensor.isEmpty() || listTarget.isEmpty() || listSink.isEmpty()) {
            return;
        }
        //Khoi tao danh sach Pi
        for (int k = 0; k < listTarget.size(); k++) {
            List<List<Integer>> Pi = new ArrayList<>();
            int target = listTarget.get(k);

            ListP.clear();
            ListParent.clear();
            //System.out.println("Target "+k + " id ="+target);
            //
            List<Integer> listParent1 = new ArrayList<>();
            int num =0;
            for (int i = 0; i < listSensor.size(); i++) {
                if (checkSensorConnectSink(listSensor.get(i), listSink)) {
                    List<Integer> list = new ArrayList<>();
                    list.add(listSensor.get(i));
                    listParent1.add(listSensor.get(i));
                    
                    if (Distance[listSensor.get(i)][N + target] <= Rs) {
                       Pi.add(list);
                    } else {
                       ListP.add(list);
                       num++;
                    }

                }
            }
            for (int j = 0;j< num;j++) {
               ListParent.add(listParent1);
            }
            

            while (!ListP.isEmpty()) {
                List<Integer> headP = ListP.get(0);
                List<Integer> headParent = ListParent.get(0);
                int lastSensor = headP.get(headP.size() - 1); // Lay phan tu cuoi cung cua head

                if (Distance[lastSensor][N + target] <= Rs) {
                    Pi.add(headP);
                    ListP.remove(0);
                    ListParent.remove(0);
                    continue;
                }

                if (headP.size() == MaxHopper) {
                    ListP.remove(0);
                    ListParent.remove(0);
                } else {
                    List<Integer> listParent = new ArrayList<>();
                    for (int j = 0; j < headParent.size(); j++) {
                        listParent.add(headParent.get(j));
                    }
                    int count =0;
                    for (int i = 0; i < listSensor.size(); i++) {
                        if (lastSensor != listSensor.get(i) && Distance[lastSensor][listSensor.get(i)] <= Rc) {

                            if (!checkPointExitInList(listSensor.get(i), headParent)) {
                                // Coppy to new Array
                                List<Integer> list = new ArrayList<>();
                                for (int j = 0; j < headP.size(); j++) {
                                    list.add(headP.get(j));
                                }
                                list.add(listSensor.get(i));
                                listParent.add(listSensor.get(i));
                                count++;

                                //Add list to P
                                ListP.add(list);
                            }

                        }

                    }
                    for (int j = 0;j< count;j++) {
                       ListParent.add(listParent);
                    }
                    ListP.remove(0);
                    ListParent.remove(0);

                }

            }
            
            ListPi.add(Pi);
        }
        
        //Dao nguoc ListPath
        System.gc();
        ListPathY.clear();
        for (int i =0 ; i< ListPi.size(); i++) {
            List<List<Integer>> Pi = ListPi.get(i);
            List<PathItem> listPath = new ArrayList<>();
            for (int j = 0 ; j< Pi.size(); j++) {
                List<Integer> p = Pi.get(j);
                PathItem path = new PathItem();
                for (int k = p.size()-1; k >= 0 ; k--) {
                    path.addElement(p.get(k));
                }
                listPath.add(path);
            }
            ListPathY.add(listPath);
        }
        
    }
    
    boolean checkPointExitInList(int point , List<Integer> listPoint ) {
        for (int i = 0 ; i < listPoint.size(); i++) {
            if (point == listPoint.get(i)) return true;
        }
        return false;
    }
    
    
    float TranferEnergy(float distance) {
        float result = Et;
        if (distance <Do) {
            result+= (Efs*distance*distance);
        } else {
            result+= (Emp*distance*distance*distance*distance);
        }
        return result;
    }
    float CaculateEnergyConsume(List<List<Integer>> ListPath,int sensor) {
        List<Integer> path;
        float result =0;
        for (int i =0 ; i < ListPath.size(); i++) {
            path = ListPath.get(i);
            for (int j =0; j<path.size();j++) {
                if (j == 0 && sensor == path.get(j)) {
                    //TH sensor la sensing node
                    result += bit*Es;
                    if (path.size()== 1) {
                        result += bit*TranferEnergy(MinDistanceSink[sensor]);
                    } else {
                        result += bit*TranferEnergy(Distance[sensor][path.get(j+1)]);
                    }
                    
                    break;
                } else if (sensor == path.get(j)) {
                    //TH sensor laf relay node
                    result += bit*Er;
                    if (j == path.size()-1) {
                        result += bit*TranferEnergy(MinDistanceSink[sensor]);
                    } else {
                        result += bit*TranferEnergy(Distance[sensor][path.get(j+1)]);
                    }
                    break;
                }
            }
            
        }
        return result;
    }
    
    int checkExitPathItemInList(PathItem item, List<CustomPathItem> listCustomAllPath) {
         List<Integer> listSensor = item.getPath();
         for (int i =0 ; i< listCustomAllPath.size(); i++) {
             CustomPathItem customPathItem = listCustomAllPath.get(i);
             List<Integer> listTempSensor = customPathItem.getPathItem().getPath();
             if (listSensor.size() == listTempSensor.size() && listSensor.get(0) == listTempSensor.get(0)
                     && listSensor.get(listSensor.size()-1) == listTempSensor.get(listTempSensor.size()-1)) {
                 int count =0;
                 for (int j =0 ; j < listSensor.size(); j++) {
                     if (listSensor.get(j) == listTempSensor.get(j)) {
                         count++;
                     } else {
                         break;
                     }
                 }
                 if (count == listSensor.size()) return i;
             }
         }
         return -1;
     }
    
     public List<List<Double>> LinearProAlgorithm3(List<List<PathItem>> listPathY, List<Integer> listSenSor,List<Integer> listTarget, double valueE0) {
        List<List<Double>> ListTime = new ArrayList<>();
        int n = listPathY.size(); // Number target
        int m = listSenSor.size(); // Number sensor
        int Vmax =0;

        System.out.println();

        if (m == 0 || n == 0) {
            return ListTime;
        }
        int totalpath =0;
        int[] v = new int[n];
        for (int i = 0; i < listPathY.size(); i++) {
            v[i] = listPathY.get(i).size();
            if (Vmax < listPathY.get(i).size()) {
                Vmax = listPathY.get(i).size();
            }
            int id = listTarget.get(i);
            System.out.println("Target "+i+ " id ="+id+" (X ="+ mListTargetNodes.get(id).getX()+" ,Y ="+ mListTargetNodes.get(id).getY()+")"+ " sizePath ="+ v[i]);
            totalpath += v[i];
        }
        System.out.println("Total Path " + totalpath);

        //Check Input
        List<CustomPathItem> ListAllPath = new ArrayList<>();
        List<List<Integer>> ListofListPathOfTarget = new ArrayList<>();
        for (int i = 0; i <listPathY.size(); i++) {
            List<PathItem> PathY = listPathY.get(i);
            List<Integer> ListPathOfTarget  = new ArrayList<>();
            for (int j =0; j < PathY.size(); j++) {
                PathItem item = PathY.get(j);
                int postion = checkExitPathItemInList(item, ListAllPath);
                if (postion == -1) {
                    List<Integer> listId = new ArrayList<>();
                    listId.add(i);
                    CustomPathItem customPathItem = new CustomPathItem(listId, item);
                    ListPathOfTarget.add(ListAllPath.size());
                    ListAllPath.add(customPathItem);
                } else {
                    CustomPathItem customPathItem = ListAllPath.get(postion);
                    customPathItem.getListId().add(i);
                    ListPathOfTarget.add(postion);
                }
            }
            // Add to List of  List Path of target
            ListofListPathOfTarget.add(ListPathOfTarget);

        }
        System.out.println("Total Path Tong hop " + ListAllPath.size());
      
        try {
            //Init model
            IloCplex cplex = new IloCplex();

            //Define variable
            int Max =ListAllPath.size();
            IloNumVar[] t = new IloNumVar[Max];
            
      
            for (int j = 0; j < Max; j++) {
                t[j] = cplex.numVar(0, Float.MAX_VALUE);
            }

            //Define Objective
            IloNumVar object = cplex.numVar(0, Float.MAX_VALUE);
            IloLinearNumExpr objective = cplex.linearNumExpr();
            objective.addTerm(1.0, object);
            
            cplex.addMaximize(objective);
            
            //Contraint
            //Energy of Sensor <= Eo
            for (int i = 0; i < m; i++) {
                IloLinearNumExpr arrayExpress = cplex.linearNumExpr();
                int sensor = listSenSor.get(i);
                for (int j = 0; j < Max; j++) {
                    CustomPathItem customPathItem = ListAllPath.get(j);
                    List<Integer> listPa = customPathItem.getPathItem().getPath();
                    float value = getEnergyConsumer(listPa, sensor);

                    arrayExpress.addTerm(value, t[j]);

                }
                cplex.addLe(arrayExpress, valueE0);
            }
            
            //Time On of target <= Object
            IloLinearNumExpr[] express = new IloLinearNumExpr[n];
            for (int j = 0; j < n; j++) {
                express[j] = cplex.linearNumExpr();
                List<Integer> listOfPathTarget = ListofListPathOfTarget.get(j);
                for (int k = 0; k <listOfPathTarget.size(); k++) {
                    express[j].addTerm(1.0, t[listOfPathTarget.get(k)]);
                }
                cplex.addLe(object,express[j]);

            }

            cplex.setParam(IloCplex.Param.Simplex.Display, 0);
            
            if (cplex.solve()) {

                System.out.println("value: " + cplex.getObjValue());
                double[] Time = new double[n];
                
                //Reduce variable =0;
                int cnt =0;
                for (int j = 0; j < ListAllPath.size();) {
                    if (cplex.getValue(t[cnt]) > 0) {
                        CustomPathItem customPathItem = ListAllPath.get(j);
                        customPathItem.setTime(cplex.getValue(t[cnt]));
                        j++;
                    } else {
                        ListAllPath.remove(j);
                    }
                    cnt++;
                }
                
                //Convert to ListY and ListTime
                //Clear data 
                for (int i =0; i < listPathY.size(); i++) {
                    List<PathItem> pathY = listPathY.get(i);
                    pathY.clear();
                    List<Double> time = new ArrayList<>();
                    ListTime.add(time);
                }
                //Coppy to ListY and ListTime
                for (int i =0 ; i < ListAllPath.size();i++) {
                    CustomPathItem customPathItem = ListAllPath.get(i);
                    List<Integer> listTagetId = customPathItem.getListId();
                    for (int j =0; j < listTagetId.size(); j++) {
                        int id = listTagetId.get(j);
                        List<Integer> path = customPathItem.getPathItem().getPath();
                        List<Integer> tempPath = new ArrayList<>();
                        
                        for (int k =0; k <path.size();k++) {
                            tempPath.add(path.get(k));
                        }
                        PathItem pathItem = new PathItem(tempPath);
                        double time = customPathItem.getTime();
                        listPathY.get(id).add(pathItem);
                        ListTime.get(id).add(time);
                    }
                }
                
                //test print result
                for (int i =0 ; i < listPathY.size();i++) {
                    List<PathItem> pathY = listPathY.get(i);
                    List<Double> time = ListTime.get(i);
                    for (int j =0; j < pathY.size(); j++) {
                        System.out.print(" "+time.get(j).doubleValue());
                    }
                    System.out.println();
                }

                //return cplex.getValue(objective);        
            } else {
                System.out.println("Problem not solved");
            }

            cplex.end();

        } catch (IloException ex) {
            Logger.getLogger("LeHieu").log(Level.SEVERE, null, ex);
        }
        //Free data
        v = null; 
        ListAllPath = null;
        ListofListPathOfTarget = null;
        return ListTime;
    }
     
    float getEnergyConsumer(List<Integer> pathYi, int sensor) {
        float result = 0;
        for (int i =0; i< pathYi.size(); i++) {
            if (i==0 && pathYi.get(i) == sensor ) {
                result += bit * Es;
                if (pathYi.size() == 1) {
                    result += bit * TranferEnergy(MinDistanceSink[sensor]);
                } else {
                    result += bit * TranferEnergy(Distance[sensor][pathYi.get(i + 1)]);
                }
                return result;
            } else if (pathYi.get(i) == sensor) {
                result += bit * Er;
                if (i == pathYi.size() - 1) {
                    result += bit * TranferEnergy(MinDistanceSink[sensor]);
                } else {
                    result += bit * TranferEnergy(Distance[sensor][pathYi.get(i + 1)]);
                }
                return result;
            }
        }
        return 0.0f;
    }
    
    public void LinearProAlgorithm4(List<CustomPathItem> listTotalPath, List<Integer> listSenSor,List<Integer> listTarget, List<EnergyItem> listEnergy) {
        int n = listTarget.size(); // Number target
        int m = listSenSor.size(); // Number sensor
        
        //Khoi tao list luu tru id target theo tung CustomPathItem
        List<List<Integer>> ListofListPathOfTarget = new ArrayList<>();
        for (int i = 0; i <n; i++) {
            List<Integer> ListPathOfTarget  = new ArrayList<>();
            ListofListPathOfTarget.add(ListPathOfTarget);
        }
        
        for (int i =0; i< listTotalPath.size();i++) {
            CustomPathItem customPathItem = listTotalPath.get(i);
            List<Integer> listIdTarget = customPathItem.getListId();
            
            for (int j =0; j < listIdTarget.size();j++) {
                int target = listIdTarget.get(j);
                ListofListPathOfTarget.get(target).add(i);
            }
        }
        
        System.out.println("Total Path Tong hop " + listTotalPath.size());
      
        try {
            //Init model
            IloCplex cplex = new IloCplex();

            //Define variable
            int Max =listTotalPath.size();
            IloNumVar[] t = new IloNumVar[Max];
            
      
            for (int j = 0; j < Max; j++) {
                t[j] = cplex.numVar(0, Float.MAX_VALUE);
            }

            //Define Objective
            IloNumVar object = cplex.numVar(0, Float.MAX_VALUE);
            IloLinearNumExpr objective = cplex.linearNumExpr();
            objective.addTerm(1.0, object);
            
            cplex.addMaximize(objective);
            
            //Contraint
            //Energy of Sensor <= Eo
            for (int i = 0; i < m; i++) {
                IloLinearNumExpr arrayExpress = cplex.linearNumExpr();
                int sensor = listSenSor.get(i);
                for (int j = 0; j < Max; j++) {
                    CustomPathItem customPathItem = listTotalPath.get(j);
                    List<Integer> listPa = customPathItem.getPathItem().getPath();
                    float value = getEnergyConsumer(listPa, sensor);

                    arrayExpress.addTerm(value, t[j]);

                }
                cplex.addLe(arrayExpress, listEnergy.get(sensor).getEnergyUse());
            }
            
            //Time On of target <= Object
            IloLinearNumExpr[] express = new IloLinearNumExpr[n];
            for (int j = 0; j < n; j++) {
                express[j] = cplex.linearNumExpr();
                List<Integer> listOfPathTarget = ListofListPathOfTarget.get(j);
                for (int k = 0; k <listOfPathTarget.size(); k++) {
                    express[j].addTerm(1.0, t[listOfPathTarget.get(k)]);
                }
                cplex.addLe(object,express[j]);

            }

            cplex.setParam(IloCplex.Param.Simplex.Display, 0);
            
            if (cplex.solve()) {

                System.out.println("value: " + cplex.getObjValue());
                double[] Time = new double[n];
                
                //Reduce variable =0;
                int cnt =0;
                for (int j = 0; j < listTotalPath.size();) {
                    if (cplex.getValue(t[cnt]) > 0) {
                        CustomPathItem customPathItem = listTotalPath.get(j);
                        customPathItem.setTime(cplex.getValue(t[cnt]));
                        j++;
                    } else {
                        listTotalPath.remove(j);
                    }
                    cnt++;
                }

                //test print result
                for (int i =0 ; i < listTotalPath.size();i++) {
                    PathItem pathY = listTotalPath.get(i).getPathItem();
                    double time = listTotalPath.get(i).getTime();
                    
                    System.out.print(" "+time);
                }

                //return cplex.getValue(objective);        
            } else {
                System.out.println("Problem not solved");
            }

            cplex.end();

        } catch (IloException ex) {
            Logger.getLogger("LeHieu").log(Level.SEVERE, null, ex);
        }
        //Free data
        ListofListPathOfTarget = null; 
        
    }
    
     public List<List<Double>> HeuriticLinearAlgorithm(List<List<PathItem>> listPathY, List<Integer> listSenSor,List<Integer> listTarget, float mEo) {
         //Create List energy 
         List<EnergyItem> listSensorEnergy = new ArrayList<>();
         for (int i =0; i < N; i++) {
             EnergyItem energyItem = new EnergyItem(i, mEo);
             listSensorEnergy.add(energyItem);
         }
         int NumSensor[] = new int[N];
         List<CustomPathItem> returnListTotalPathItem = new ArrayList<>();

         
        do {
            //Tim List dot dau
            List<CustomPathItem> ListTotalPathItem = findListHeuristicPathItem(listPathY, listSensorEnergy, listSenSor, NumSensor);
            //Calculate Linear Programing and update ListTotalPathItem
            List<Integer> tempListSensor = new ArrayList<>();
            FindListSensorInAllPath2(ListTotalPathItem,tempListSensor);
            LinearProAlgorithm4(ListTotalPathItem, tempListSensor, listTarget, listSensorEnergy);

            //Update energy
            UpdateEnergy(listSensorEnergy, ListTotalPathItem);

            //Remove SomePath has Energy of Sensor =0
            ReduceListYFollowEnergy(listPathY, listSensorEnergy);

            //Add tempList to templeResult List
            AddToResultList(returnListTotalPathItem, ListTotalPathItem);
            ListTotalPathItem.clear();
        } while (checkExitListY_InTarget(listPathY, listTarget));  // Finish
             
         //Delete old data
         List<List<Double>> tempReturnListTi = new ArrayList<>();
         for (int i =0; i < listPathY.size();i++) {
             List<PathItem> pathY = listPathY.get(i);
             pathY.clear();
             List<Double> tempListTi= new ArrayList<>();
             tempReturnListTi.add(tempListTi);
         }
         
         //Coppy data from returnListTotalPathItem
        for (int i = 0; i < returnListTotalPathItem.size(); i++) {
            CustomPathItem customPathItem = returnListTotalPathItem.get(i);
            List<Integer> listTagetId = customPathItem.getListId();
            for (int j = 0; j < listTagetId.size(); j++) {
                int id = listTagetId.get(j);
                List<Integer> path = customPathItem.getPathItem().getPath();
                List<Integer> tempPath = new ArrayList<>();

                for (int k = 0; k < path.size(); k++) {
                    tempPath.add(path.get(k));
                }
                PathItem pathItem = new PathItem(tempPath);
                double time = customPathItem.getTime();
                listPathY.get(id).add(pathItem);
                tempReturnListTi.get(id).add(time);
            }
        }
         
         
         return tempReturnListTi;
     }
     
     void UpdateEnergy(List<EnergyItem> listSensorEnergy, List<CustomPathItem> tempListTotalPath) {

        for (int i = 0; i < tempListTotalPath.size(); i++) {
            CustomPathItem customPathItem = tempListTotalPath.get(i);

            double time = customPathItem.getTime();
            PathItem pathY = customPathItem.getPathItem();
            List<Integer> listPath = pathY.getPath();
            for (int k = 0; k < listPath.size(); k++) {
                int sensor = listPath.get(k);
                float value = getEnergyConsumer(listPath, sensor);
                //Giam nang luong trong list Energy Sensor

                EnergyItem energyItem = listSensorEnergy.get(sensor);
                energyItem.subEnergyUse((float) (value * time));

            }

        }
    }
     
     
     void ReduceListYFollowEnergy(List<List<PathItem>> listPathY, List<EnergyItem> listSensorEnergy) {
         for (int i =0; i < listPathY.size(); i++) {
             List<PathItem> listY = listPathY.get(i);
             boolean isZero;
             for (int j =0; j < listY.size(); ) {
                 isZero = false;
                 List<Integer> list = listY.get(j).getPath();
                 for (int k =0; k < list.size(); k++) {
                     int sensor = list.get(k);
                     if (listSensorEnergy.get(sensor).getEnergyUse() < SensorUtility.mOffset ) {
                         isZero = true;
                         break;
                     }
                 }
                 
                 if (isZero) {
                     listY.remove(j);
                 } else j++;

             }
         }
     }
    
     boolean checkExitListY_InTarget(List<List<PathItem>> listPathY , List<Integer> listTarget) {
         for (int i =0; i< listTarget.size(); i++) {
             int target = listTarget.get(i);
             List<PathItem> listY = listPathY.get(i);
             
             if (listY.isEmpty()) return false;
         }
         return true;
     }
     
     void AddToResultList(List<CustomPathItem> tempReturnTotalPath, List<CustomPathItem> tempListTotalPath) {
         
         for (int i =0; i < tempListTotalPath.size(); i++) {
             CustomPathItem customPathItem = tempListTotalPath.get(i);
             tempReturnTotalPath.add(customPathItem);
         }
         
         
     }
     public List<CustomPathItem> findListHeuristicPathItem(List<List<PathItem>> listPathY,List<EnergyItem> listSensorEnergy, List<Integer> listSenSor, int NumSensor[]) {
         //reset Number Sensor 
         for (int i =0 ; i< N; i++) {
             NumSensor[i] = 0;
         }
         List<CustomPathItem> ListTotalPathItem = new ArrayList<>();
         
         List<HeuristicItem> listRi = new ArrayList<>();
         boolean isFirst = true;
       
         for (int i = 0; i < listPathY.size(); i++) {
             List<PathItem> PathY = listPathY.get(i);
             listRi.clear();
             if (!PathY.isEmpty()) {
                 //Calculate Ri of List Path
                 for (int j = 0; j < PathY.size(); j++) {
                     PathItem pathItem = PathY.get(j);
                     HeuristicItem heuristicItem = new HeuristicItem();
                     heuristicItem.setId(j);
                     float ri = CaculateRiValue(listSensorEnergy, pathItem, NumSensor, isFirst);
                     heuristicItem.setValue(ri);
                     listRi.add(heuristicItem);
                 }

                 //Sort lại list HeuristicItem theo thu tu tang dan
                 Collections.sort(listRi, new Comparator<HeuristicItem>() {
                     @Override
                     public int compare(HeuristicItem o1, HeuristicItem o2) {
                         float size1 = o1.getValue();
                         float size2 = o2.getValue();

                         return Float.compare(size1, size2);
                     }

                 });
                 
                 //Lua chon cac path theo dieu kien
                 int last = 0;
                 for (int j =0; j < listRi.size(); j++) {
                     int posId = listRi.get(j).getId();
                     float ri = listRi.get(j).getValue();
                     
                     //Add tem cac dk can thiet
                     if (j < SensorUtility.mKmax) {
                         PathItem item = PathY.get(posId);
                         int post = checkExitPathItemInList(item, ListTotalPathItem);
                         
                         if (post == -1) {
                              List<Integer> listId = new ArrayList<>();
                              listId.add(i);
                              CustomPathItem customPathItem = new CustomPathItem(listId, item);
                              ListTotalPathItem.add(customPathItem);
                         } else {
                             CustomPathItem customPathItem = ListTotalPathItem.get(post);
                             customPathItem.getListId().add(i);
                         }
                         last = j;
                     } else {
                         break;
                     }
                     
                     
                 }
                 
                  //Update so lan xuat hien cau sensor
                 for (int j =0; j <= last;j++) {
                     int posId = listRi.get(j).getId();
                     List<Integer> path = PathY.get(posId).getPath();
                     for (int k = 0; k < path.size(); k++) {
                         int sensor = path.get(k);
                         NumSensor[sensor] ++;
                     }
                     
                 }
                 
                 //Remove PathItem in ListY
                 List<Integer> listIdRemove = new ArrayList<>();
                 for (int j =0; j <=last;j++) {
                     int posId = listRi.get(j).getId();
                     listIdRemove.add(posId);
                 }
                 Collections.sort(listIdRemove);
                 for (int j =0; j < listIdRemove.size();j++) {
                     int id = listIdRemove.get(j);
                     int index = id - j;
                     if (index <= 0) {
                         PathY.remove(0);
                     } else {
                         PathY.remove(index);
                     }
                 }
                 
                

                 isFirst = false;
             }

         }
         return ListTotalPathItem;
         
     }
    
    
     float CaculateRiValue(List<EnergyItem> listSensorEnergy, PathItem pathItem, int NumSensor[], boolean isFirst) {
         List<Integer> listSensorPath = pathItem.getPath();
         float Emax = 0; // nang luong lon nhat cua path
         float Emin = SensorUtility.mEoValue; // nang luong nho nhat cua path
         float Etotal = 0; // Tong nang luong con lai
         float Ec = 0; //tong nang luong tieu hao cua path
         //Find Emax, Emin, Etotal
         for (int i =0; i < listSensorPath.size(); i++) {
             int sensor = listSensorPath.get(i);
             float tempE = listSensorEnergy.get(sensor).getEnergyUse();
             
             //Etotal
             Etotal += tempE;
             //Emax
             if (Emax < tempE) Emax = tempE;
             //Emin
             if (Emin > tempE) Emin = tempE;
             
             //Ec
             Ec += getEnergyConsumer(listSensorPath, sensor);
             
         }
         
         //Check
         if (isFirst) {
             return (SensorUtility.mGamma1*(Ec/Etotal) + SensorUtility.mGamma2*(Emax/Emin));
             
         } else {
             //
             float Total =0;
             for (int i = 0; i < listSensorPath.size(); i++) {
                 int sensor = listSensorPath.get(i);
                 int num = NumSensor[sensor];
                 num++;
                 Total += (num /(num+1));
             }
             
             return (SensorUtility.mGamma1*(Ec/Etotal) + SensorUtility.mGamma2*(Emax/Emin) + (1-SensorUtility.mGamma1-SensorUtility.mGamma2)*Total);
             
         }

     }
     
     
    void FindListSensorInAllPath2(List<CustomPathItem> returnListTotalPath, List<Integer> listSenSor) {
        boolean checkExit[];
        checkExit = new boolean[N];
        for (int i = 0; i < returnListTotalPath.size(); i++) {
            PathItem path = returnListTotalPath.get(i).getPathItem();
            List<Integer> list = path.getPath();
            for (int k = 0; k < list.size(); k++) {
                checkExit[list.get(k)] = true;
            }
        }
        
        listSenSor.clear();
        for (int i =0; i < checkExit.length; i++) {
            if (checkExit[i]) {
                listSenSor.add(i);
            }
        }
    }
    
    public void CoppyToListSensor() {
        mListofListPath.clear();
        mListofListPath = resultListY;
        SensorUtility.mListofListPathTime = resultListTi;
//        for (int i =0; i < mListofListPath.size();i++) {
//            List<PathItem> listPath = mListofListPath.get(i);
//            List<Double> listTime = resultListTi.get(i);
//            for (int j = 0; j <listPath.size();j++) {
//                PathItem path= listPath.get(j);
//                Double time = listTime.get(j);
//                List<Integer> listPoint = path.getPath();
//                for (int k = 0; k < listPoint.size(); k++) {
//                    int point = listPoint.get(k);
//                    ListEnergyUsing[point] += (getEnergyConsumer(listPoint, point) * time.floatValue());
//                }
//            }
//        }
//        System.out.println("Nang luong cua cac Sensor :--------------");
//        for (int i =0 ; i < ListEnergySensor.length;i++) {
//            System.out.print(ListEnergyUsing[i]/1000000000+" ");
//        }
        
       //Create: List All Path and Time
       List<CustomPathItem> ListAllPathItem = new ArrayList<>();
       for (int i = 0; i< mListofListPath.size(); i++) {
           List<PathItem> PathY = mListofListPath.get(i);
           for (int j =0; j < PathY.size(); j++) {
                PathItem item = PathY.get(j);
                int postion = checkExitPathItemInList(item, ListAllPathItem);
                if (postion == -1) {
                    List<Integer> listId = new ArrayList<>();
                    listId.add(i);
                    CustomPathItem customPathItem = new CustomPathItem(listId, item);
                    customPathItem.setTime(mListofListPathTime.get(i).get(j));
                    ListAllPathItem.add(customPathItem);
                } else {
                    CustomPathItem customPathItem = ListAllPathItem.get(postion);
                    customPathItem.getListId().add(i);
                }
           }
       }
       
       //Calculate Energy using of Sensor
        for (int j = 0; j < ListAllPathItem.size(); j++) {
            PathItem path = ListAllPathItem.get(j).getPathItem();
            double time = ListAllPathItem.get(j).getTime();
            List<Integer> listPoint = path.getPath();
            for (int k = 0; k < listPoint.size(); k++) {
                int point = listPoint.get(k);
                ListEnergyUsing[point] += (getEnergyConsumer(listPoint, point) * time);
            }
        }
        System.out.println("Nang luong cua cac Sensor :--------------");
        for (int i =0 ; i < ListEnergySensor.length;i++) {
            System.out.print(ListEnergyUsing[i]/1000000000+" ");
        }
        System.out.println();
    }
       
    public void runAlgorithm() {
        List<Integer> listSensor = new ArrayList<>();
        for (int i = 0; i < mListSensorNodes.size(); i++) {
            listSensor.add(i);
        }
        List<Integer> listTarget = new ArrayList<>();
        for (int i = 0; i < mListTargetNodes.size(); i++) {
            listTarget.add(i);
        }
        List<Integer> listSink = new ArrayList<>();
        for (int i = 0; i < mListSinkNodes.size(); i++) {
            listSink.add(i);
        }
        
        FloatPointItem UpLeftCornerPoint = new FloatPointItem(0,0);
        FloatPointItem DownRightCornerPoint = new FloatPointItem(SensorUtility.numberRow,SensorUtility.numberColum);
        
        List<List<List<PathItem>>> ListOfListY = new ArrayList<>();
        List<List<List<Double>>> ListOfListTi = new ArrayList<>();

        for (int i =0; i< Anpha ;i++) {
            List<List<PathItem>> tempReturnListY = new ArrayList<>();
            List<List<Double>> tempReturnListTi = new ArrayList<>();
            for (int j = 0; j< T;j++) {
                List<PathItem> pathY = new ArrayList<>();
                List<Double> timeY = new ArrayList<>(); 
                tempReturnListY.add(pathY);
                tempReturnListTi.add(timeY);
            }
            
            
            DiviceNetworkFollowWidth(UpLeftCornerPoint,DownRightCornerPoint,i,tempReturnListY,tempReturnListTi);
            if (!tempReturnListY.isEmpty() && !tempReturnListTi.isEmpty()) {
                if (CheckEnergyMoreThanEo3(tempReturnListY, tempReturnListTi)) {
                    isWidthOptimal = false;
                    ListOfListY.add(tempReturnListY);
                    ListOfListTi.add(tempReturnListTi);
                } else {
                    isWidthOptimal = true;
                    ListOfListY.clear();
                    ListOfListTi.clear();
                    ListOfListY.add(tempReturnListY);
                    ListOfListTi.add(tempReturnListTi);
                    break;
                }

            }
        }
        if (!isWidthOptimal) {
            Combining_All_Division3(ListOfListY,ListOfListTi,resultListY,resultListTi);
        } else {
            System.out.println("Found case optimize follow width");
            resultListY = ListOfListY.get(0);
            resultListTi = ListOfListTi.get(0);
            isWidthOptimal = false;
        }
        
        //Free data
        ListOfListY = null;
        ListOfListTi = null;

    }
    
    public  void DiviceNetworkFollowWidth(FloatPointItem UpLeftCornerPoint, FloatPointItem DownRightCornerPoint, int divisons, List<List<PathItem>> returnListY, List<List<Double>> returnListTi) {
        FloatPointItem upPoint = new FloatPointItem();
        FloatPointItem downPoint = new FloatPointItem();
        
        List<List<List<PathItem>>> tempListOfListY = new ArrayList<>();
        List<List<List<Double>>> tempListOfListTi = new ArrayList<>();
        List<List<PathItem>> tempListY;
        List<List<Double>> tempListT;
        
        List<List<List<PathItem>>> temp2ListOfListY = new ArrayList<>();
        List<List<List<Double>>> temp2ListOfListTi = new ArrayList<>();

        
        float PostionX = UpLeftCornerPoint.getX();
        float MaxPostionX = DownRightCornerPoint.getX();
        boolean isFirstBlock = true;
        
        
        while (PostionX < MaxPostionX)  {
            //Set upoint and downpoint of Block
            if (isFirstBlock && divisons != 0) {
                //Start Point of Block independent of offset value
                upPoint.setXY(PostionX, UpLeftCornerPoint.getY());
                PostionX += divisons * 2 * R;
                if (PostionX >= MaxPostionX) PostionX = MaxPostionX;
                downPoint.setXY(PostionX, DownRightCornerPoint.getY());
                isFirstBlock = false;
            } else {
                
                upPoint.setXY(PostionX, UpLeftCornerPoint.getY());
                PostionX += Anpha*2 * R;
                if (PostionX >= MaxPostionX) PostionX = MaxPostionX;
                downPoint.setXY(PostionX, DownRightCornerPoint.getY());
            }
            
            //Caculate result of the Block foreach devisions
            for (int i =0; i< Anpha;i++) {
                List<List<PathItem>> temp2returnListX = new ArrayList<>();
                List<List<Double>> temp2returnListTi = new ArrayList<>();
                for (int j = 0; j< T;j++) {
                    List<PathItem> pathY = new ArrayList<>();
                    List<Double> timeY = new ArrayList<>(); 
                    temp2returnListX.add(pathY);
                    temp2returnListTi.add(timeY);
                }
                DiviceNetWorkFollowHeight(upPoint,downPoint,i,temp2returnListX,temp2returnListTi);
                if (!temp2returnListX.isEmpty() && !temp2returnListTi.isEmpty()) {
                    if (CheckEnergyMoreThanEo3(temp2returnListX, temp2returnListTi)) {
                        isHeightOptimal = false;
                        temp2ListOfListY.add(temp2returnListX);
                        temp2ListOfListTi.add(temp2returnListTi);
                    } else {
                        isHeightOptimal = true; //Tim thay nghiem toi uu
                        temp2ListOfListY.clear();
                        temp2ListOfListTi.clear();
                        temp2ListOfListY.add(temp2returnListX);
                        temp2ListOfListTi.add(temp2returnListTi);
                        break;
                    }
                }
            }
            tempListY = new ArrayList<>();
            tempListT = new ArrayList<>();
            
            for (int j = 0; j< T;j++) {
                    List<PathItem> pathY = new ArrayList<>();
                    List<Double> timeY = new ArrayList<>(); 
                    tempListY.add(pathY);
                    tempListT.add(timeY);
            }
            if (!isHeightOptimal) {
                Combining_All_Division3(temp2ListOfListY, temp2ListOfListTi, tempListY, tempListT);
            } else {
                System.out.println("Found case optimize follow height");
                tempListY = temp2ListOfListY.get(0);
                tempListT = temp2ListOfListTi.get(0);
                isHeightOptimal = false;
            }
            temp2ListOfListY.clear();
            temp2ListOfListTi.clear();
            if (!tempListY.isEmpty() && !tempListT.isEmpty()) {
               tempListOfListY.add(tempListY);
               tempListOfListTi.add(tempListT);
            }
        }
        
        //Combining all strips follow the with of netwwork
        Combining_All_Strips(tempListOfListY, tempListOfListTi, returnListY, returnListTi);
        
        //Free Data
        tempListOfListY = null;
        tempListOfListTi = null;
        tempListY = null;
        tempListT = null;
        temp2ListOfListY = null;
        temp2ListOfListTi = null;
    }
    
    public void DiviceNetWorkFollowHeight(FloatPointItem UpLeftCornerPoint, FloatPointItem DownRightCornerPoint, int division, List<List<PathItem>> returnListY, List<List<Double>> returnListTi) {
        
        FloatPointItem upPoint = new FloatPointItem();
        FloatPointItem downPoint = new FloatPointItem();
        
        List<List<List<PathItem>>> tempListOfListY = new ArrayList<>();
        List<List<List<Double>>> tempListOfListTi = new ArrayList<>();
        List<List<PathItem>> tempListY;
        List<List<Double>> tempListTi;
        

        float PostionY = UpLeftCornerPoint.getY();
        float MaxPostionY = DownRightCornerPoint.getY();
        boolean isFirstBlock = true;
        
        while (PostionY < MaxPostionY)  {
            //Set upoint and downpoint of Block
            if (isFirstBlock && division != 0) {
                //Start Point of Block independent of divison value
                upPoint.setXY(UpLeftCornerPoint.getX(), PostionY);
                PostionY += division * 2 * R;
                if (PostionY >= MaxPostionY) PostionY = MaxPostionY;
                downPoint.setXY(DownRightCornerPoint.getX(), PostionY);
                isFirstBlock = false;
            } else {
                
                upPoint.setXY(UpLeftCornerPoint.getX(), PostionY);
                PostionY += Anpha*2 * R;
                if (PostionY >= MaxPostionY) PostionY = MaxPostionY;
                downPoint.setXY(DownRightCornerPoint.getX(), PostionY);
            }

            //Find ListSensor in Block
            List<Integer> tempListSensor = FindListSensor(upPoint,downPoint);
            List<Integer> tempListTarget = FindListTarget(upPoint, downPoint);
            List<Integer> tempListSink = FindListSink(upPoint, downPoint);
            //showViewTest(tempListSensor);
            
            List<List<PathItem>> ListPathY = new ArrayList<>();
            List<List<Double>> ListTi;
            int postion = CheckExitListTargetInSaveList(tempListTarget, tempListSink);
            //int postion = -1;
            if (postion == -1) {
                Finding_CCP2(tempListSensor, tempListTarget, tempListSink, ListPathY);

                //ListTi = LinearProAlgorithm3(ListPathY, tempListSensor,tempListTarget, SensorUtility.mEoValue);
                ListTi = HeuriticLinearAlgorithm(ListPathY, tempListSensor, tempListTarget, SensorUtility.mEoValue);
                reduceListPathYi(ListPathY, ListTi);
                CoppyListToSave(ListPathY, ListTi, tempListTarget, tempListSink);
            } else {
                ListTi = new ArrayList<>();
                GetListFromSave(ListPathY, ListTi, postion);

            }
            
            //Init tempListTi and tempListY
            tempListY = new ArrayList<>();
            tempListTi = new ArrayList<>();
            for (int i =0 ; i< T; i++) {
                List<PathItem> Yi = new ArrayList<>();
                tempListY.add(Yi);
                
                //Ti
                List<Double> Ty = new ArrayList<>();
                tempListTi.add(Ty);
            }
            
            //Add data from  ListPathY and List Ti
            if (ListPathY.size() == tempListTarget.size()) {
                for (int i = 0; i < tempListTarget.size(); i++) {
                    int pos = tempListTarget.get(i);
                    tempListY.remove(pos);
                    tempListY.add(pos, ListPathY.get(i));

                    tempListTi.remove(pos);
                    tempListTi.add(pos, ListTi.get(i));
                }
            }
            
            //Add result of Block
            if (!tempListTarget.isEmpty() && !ListPathY.isEmpty()) {
                tempListOfListY.add(tempListY);
                tempListOfListTi.add(tempListTi);
            } else if (!tempListTarget.isEmpty() && tempListY.isEmpty()) {
                System.err.println("Khong ton tai duong di");
            }
            
            ListPathY = null;
            ListTi = null;

        }
        Combining_All_Strips(tempListOfListY, tempListOfListTi, returnListY, returnListTi);
        
        //Free Data
        tempListOfListY = null;
        tempListOfListTi = null;
        tempListY = null; 
        tempListTi = null;
        upPoint = null;
        downPoint = null;
    }
    
    void reduceListPathYi(List<List<PathItem>> ListPathY, List<List<Double>> ListTimeY) {
        for (int i = 0; i< ListPathY.size(); i++) {
            List<PathItem> pathY = ListPathY.get(i);
            List<Double> timeListY  = ListTimeY.get(i);
            // Giam cac TH TY = 0;
            for (int j =0; j< timeListY.size();) {
                if (timeListY.get(j) < 0.00001d) {
                    timeListY.remove(j);
                    pathY.remove(j);
                } else {
                    j++;
                }
                
            }
            
        }
        
        
    }
    
    public List<Integer> FindListSensor(FloatPointItem UpLeftCornerPoint, FloatPointItem DownRightCornerPoint) {
        List<Integer> resultListSensor = new ArrayList<>();
        float Xmax,Xmin,Ymax,Ymin;
        Xmin = UpLeftCornerPoint.getX() - R;
        Xmax=  DownRightCornerPoint.getX() + R;
        Ymin = UpLeftCornerPoint.getY() - R;
        Ymax = DownRightCornerPoint.getY() +R;
        
        for (int i = 0 ;i<mListSensorNodes.size(); i++) {
            if (mListSensorNodes.get(i).getX() >= Xmin && mListSensorNodes.get(i).getX() < Xmax && mListSensorNodes.get(i).getY() >= Ymin && mListSensorNodes.get(i).getY() < Ymax ) {
                resultListSensor.add(i);
            }
        }
        return resultListSensor;        
    }
    
    public List<Integer> FindListTarget(FloatPointItem UpLeftCornerPoint, FloatPointItem DownRightCornerPoint) {
        List<Integer> resultListTarget = new ArrayList<>();
        float Xmax,Xmin,Ymax,Ymin;
        Xmin = UpLeftCornerPoint.getX();
        Xmax=  DownRightCornerPoint.getX();
        Ymin = UpLeftCornerPoint.getY();
        Ymax = DownRightCornerPoint.getY();
        
        for (int i = 0 ;i<mListTargetNodes.size(); i++) {
            if (mListTargetNodes.get(i).getX() >= Xmin && mListTargetNodes.get(i).getX() < Xmax && mListTargetNodes.get(i).getY() >= Ymin && mListTargetNodes.get(i).getY() < Ymax ) {
                resultListTarget.add(i);
            }
        }
        return resultListTarget;        
    }
    
    public List<Integer> FindListSink(FloatPointItem UpLeftCornerPoint, FloatPointItem DownRightCornerPoint) {
        List<Integer> resultListSink = new ArrayList<>();
        float Xmax,Xmin,Ymax,Ymin;
        Xmin = UpLeftCornerPoint.getX() - R-Rc;
        Xmax=  DownRightCornerPoint.getX() + R+Rc;
        Ymin = UpLeftCornerPoint.getY() - R-Rc;
        Ymax = DownRightCornerPoint.getY() +R+Rc;
        
        for (int i = 0 ;i<mListSinkNodes.size(); i++) {
            if (mListSinkNodes.get(i).getX() >= Xmin && mListSinkNodes.get(i).getX() < Xmax && mListSinkNodes.get(i).getY() >= Ymin && mListSinkNodes.get(i).getY() < Ymax ) {
                resultListSink.add(i);
            }
        }
        return resultListSink;        
    }
    
    public void Combining_All_Strips(List<List<List<PathItem>>> ListOfListY, List<List<List<Double>>> ListOfListTi, List<List<PathItem>> returnListY, List<List<Double>> returnListTi) {
        int Strip = ListOfListY.size();
        double Min = Double.MAX_VALUE;
        //Create ListT Ascending
        
        for(int i =0; i<Strip ; i++){
            List<List<PathItem>> ListPathY = ListOfListY.get(i);
            List<List<Double>> ListTimeTi = ListOfListTi.get(i);
             for (int j = 0; j< ListPathY.size();j++) {
                 List<PathItem> pathY = ListPathY.get(j);
                 List<PathItem> retPathY = returnListY.get(j);
                 List<Double> timeY = ListTimeTi.get(j);
                 List<Double> reTimeY = returnListTi.get(j);
                 
                 unionListY(pathY,timeY,retPathY,reTimeY);
                 
             }
            
            
        }

    }
    
    void unionListY(List<PathItem> inputListY ,List<Double> inputTime, List<PathItem> outputListY, List<Double> outputTime) {
        if (outputListY.isEmpty()) {
            for (int i =0; i< inputListY.size();i++) {
                outputListY.add(inputListY.get(i));
                outputTime.add(inputTime.get(i));
            }
            
        } else {
            for (int i =0; i< inputListY.size();i++) {
                int pos = CheckExitPath(inputListY.get(i), outputListY);
                if ( pos == -1) {
                   outputListY.add(inputListY.get(i));
                   outputTime.add(inputTime.get(i));
                } else {
                   double firstTime = outputTime.get(pos);
                   double second = inputTime.get(i);
                   outputTime.remove(pos);
                   outputTime.add(pos, firstTime+second);
                }
                
            }
            
        }
        
    }
    
    int CheckExitPath(PathItem item , List<PathItem> list) {
        for (int i =0; i < list.size(); i++) {
            if (PathItem.compareSame(item, list.get(i))) {
                return i;
            }
        }
        return -1;
    }

    //Ghep Xik U Xin....
    void unionXi(List<Integer> listX, List<Integer> resultListX) {
        int n = resultListX.size();
        boolean isExit;
        for (int i = 0; i < listX.size(); i++) {
            isExit = false;
            for (int j = 0; j < n; j++) {
                if (Objects.equals(listX.get(i), resultListX.get(j))) {
                    isExit = true;
                    break;
                }
            }
            if (!isExit) {
                resultListX.add(listX.get(i));
            }
        }
    }
    

    //Dua ra vi tri corver segment cua list
    int getPostionOfList(List<Double> listT, double starTime, double endTime) {
        for (int i = 0; i < listT.size(); i++) {
            if (i == 0) {
                if (listT.get(0) >= endTime) {
                    return 0;
                }
            } else {
                if (listT.get(i - 1) <= starTime && listT.get(i) >= endTime) {
                    return i;
                }
            }
        }
        return 0;
    }
    
    public void Combining_All_Division3(List<List<List<PathItem>>> ListOfListY, List<List<List<Double>>> ListOfListT, List<List<PathItem>> returnListY, List<List<Double>> returnListTi) {

        for (int i = 0; i< ListOfListY.size();i++) {
          List<List<PathItem>> tempListPathY = ListOfListY.get(i);
          List<List<Double>>  tempListTi  =ListOfListT.get(i);
          for (int j =0 ; j< tempListPathY.size();j++) {
                 List<PathItem> pathY = tempListPathY.get(j);
                 List<PathItem> retPathY = returnListY.get(j);
                 List<Double> timeY = tempListTi.get(j);
                 List<Double> reTimeY = returnListTi.get(j);
                 
                 unionListY(pathY, timeY, retPathY, reTimeY);
              
          }
       }
       // Divice Time to (mLvalue)
       for (int i = 0; i < returnListTi.size(); i++) {
           List<Double> reTimeY = returnListTi.get(i);
           for (int j =0; j < reTimeY.size();j++) {
            Double x = reTimeY.get(j)/(Anpha);
            reTimeY.remove(j);
            reTimeY.add(j, x);
           }
        }
       
       //Tim list sensor in all Path
       List<Integer> listSensorInAllPath = new ArrayList<>();
       FindListSensorInAllPath(returnListY,listSensorInAllPath);
       
       if (listSensorInAllPath.size() == 0) return;
       //Tao list Energy tuong ung voi cac sensor      
       List<EnergyItem> listEnergy = new ArrayList<EnergyItem>();
       for (int i = 0; i < listSensorInAllPath.size();i++) {
           EnergyItem energyItem = new EnergyItem(listSensorInAllPath.get(i), 0);
           listEnergy.add(energyItem);
       }
       
       //Create: List All Path and Time
       List<CustomPathItem> ListAllPathItem = new ArrayList<>();
       for (int i = 0; i< returnListY.size(); i++) {
           List<PathItem> PathY = returnListY.get(i);
           for (int j =0; j < PathY.size(); j++) {
                PathItem item = PathY.get(j);
                int postion = checkExitPathItemInList(item, ListAllPathItem);
                if (postion == -1) {
                    List<Integer> listId = new ArrayList<>();
                    listId.add(i);
                    CustomPathItem customPathItem = new CustomPathItem(listId, item);
                    customPathItem.setTime(returnListTi.get(i).get(j));
                    ListAllPathItem.add(customPathItem);
                } else {
                    CustomPathItem customPathItem = ListAllPathItem.get(postion);
                    customPathItem.getListId().add(i);
                }
           }
       }
       
       //Calculate Energy using of Sensor
       for (int i =0; i < listSensorInAllPath.size();i++) {
           int sensor = listSensorInAllPath.get(i);
           EnergyItem energyItem = listEnergy.get(i);
           for (int j =0; j < ListAllPathItem.size();j++) {
                   PathItem path = ListAllPathItem.get(j).getPathItem();
                   Double time = ListAllPathItem.get(j).getTime();

                   float energyUse = (float)(getEnergyConsumer(path.getPath(), sensor) * time.doubleValue());
                   
                   if (energyUse > 0) {
                       energyItem.addEnergyUse(energyUse);
                       energyItem.addPostion(0, j);
                   }
                   

           }

       }
       
       //Sort cac phan tu Energy giam dan
       Collections.sort(listEnergy, new Comparator<EnergyItem>(){
            @Override
               public int compare(EnergyItem o1, EnergyItem o2) {
                   float size1 = o1.getEnergyUse();
                   float size2 = o2.getEnergyUse();
                   
                   return Float.compare(size2, size1);
               }
            
        });
        float MaxEnergyInList = listEnergy.get(0).getEnergyUse();
        if (MaxEnergyInList <= SensorUtility.mEoValue) {
            //TH1 : Energy sau khi chia cho L (Anpha) khong lon hon E0
            return ;

        } else {
            //TH2 Ton tai Energy cua 1 node lon hon Eo sau khi chia cho Anpha (L)
            //Get list Energy lon hon E0
            List<EnergyItem> listEnergyEo = new ArrayList<EnergyItem>();
            listEnergyEo.clear();
            for (int i =0; i < listEnergy.size();i++) {
                EnergyItem energyItem = listEnergy.get(i);
                if (energyItem.getEnergyUse() > SensorUtility.mEoValue) {
                    listEnergyEo.add(energyItem);
                } else {
                    break; // reason : listEnergy sorted by descending order
                }
            }
            
            CalculateReduceTime3(ListAllPathItem,listEnergyEo);
            //Update List returnY and Ti
            //Convert to ListY and ListTime
            //Clear data 
            for (int i = 0; i < returnListY.size(); i++) {
                List<PathItem> pathY = returnListY.get(i);
                pathY.clear();
                List<Double> time = returnListTi.get(i);
                time.clear();
            }
            //Coppy to ListY and ListTime
            for (int i = 0; i < ListAllPathItem.size(); i++) {
                CustomPathItem customPathItem = ListAllPathItem.get(i);
                List<Integer> listTagetId = customPathItem.getListId();
                for (int j = 0; j < listTagetId.size(); j++) {
                    int id = listTagetId.get(j);
                    List<Integer> path = customPathItem.getPathItem().getPath();
                    List<Integer> tempPath = new ArrayList<>();

                    for (int k = 0; k < path.size(); k++) {
                        tempPath.add(path.get(k));
                    }
                    PathItem pathItem = new PathItem(tempPath);
                    double time = customPathItem.getTime();
                    returnListY.get(id).add(pathItem);
                    returnListTi.get(id).add(time);
                }
            }
            
            
        }
         listEnergy = null;

    }
    
    boolean CheckEnergyMoreThanEo3(List<List<PathItem>> returnListY, List<List<Double>> returnListTi) {
        //Test
       //if (true) return true;
         //Tim list sensor in all Path
       List<Integer> listSensorInAllPath = new ArrayList<>();
       FindListSensorInAllPath(returnListY,listSensorInAllPath);
       
       if (listSensorInAllPath.isEmpty()) return false;
       //Tao list Energy tuong ung voi cac sensor      
       List<EnergyItem> listEnergy = new ArrayList<>();
       for (int i = 0; i < listSensorInAllPath.size();i++) {
           EnergyItem energyItem = new EnergyItem(listSensorInAllPath.get(i), 0);
           listEnergy.add(energyItem);
       }
       
       //Create: List All Path and Time
       List<CustomPathItem> ListAllPathItem = new ArrayList<>();
       for (int i = 0; i< returnListY.size(); i++) {
           List<PathItem> PathY = returnListY.get(i);
           for (int j =0; j < PathY.size(); j++) {
                PathItem item = PathY.get(j);
                int postion = checkExitPathItemInList(item, ListAllPathItem);
                if (postion == -1) {
                    List<Integer> listId = new ArrayList<>();
                    listId.add(i);
                    CustomPathItem customPathItem = new CustomPathItem(listId, item);
                    customPathItem.setTime(returnListTi.get(i).get(j));
                    ListAllPathItem.add(customPathItem);
                } else {
                    CustomPathItem customPathItem = ListAllPathItem.get(postion);
                    customPathItem.getListId().add(i);
                }
           }
       }
       
       //Calculate Energy using of Sensor
       for (int i =0; i < listSensorInAllPath.size();i++) {
           int sensor = listSensorInAllPath.get(i);
           for (int j =0; j < ListAllPathItem.size();j++) {
                   PathItem path = ListAllPathItem.get(j).getPathItem();
                   Double time = ListAllPathItem.get(j).getTime();
                   EnergyItem energyItem = listEnergy.get(i);
                   float energyUse = (float)(getEnergyConsumer(path.getPath(), sensor) * time);
                   
                   if (energyUse > 0) {
                       energyItem.addEnergyUse(energyUse);
                   }
                   

           }

       }
       //Check exit Energy > Eo
        for (int i = 0; i < listEnergy.size(); i++) {
            float MaxEnergyInList = listEnergy.get(i).getEnergyUse();
            if (MaxEnergyInList > SensorUtility.mEoValue +SensorUtility.mOffset) {
                return true;
            }
        }
        ListAllPathItem = null;
        return false;
    }
    
    void CalculateReduceTime3(List<CustomPathItem> listAllPathItem, List<EnergyItem> listEnergy) {
        float MaxEnergyInList = listEnergy.get(0).getEnergyUse();
        
        
        while (MaxEnergyInList > SensorUtility.mEoValue) {
            float ratio = SensorUtility.mEoValue/MaxEnergyInList;
            //Get list vi tri các Path chứa sensor
            List<Integer> listPosPath = listEnergy.get(0).getPosPathList();
            int sensor = listEnergy.get(0).getId();
            
            //Update Energy of Sensor in ListEnergy
            for (int i =0; i< listPosPath.size(); i++) {
                CustomPathItem customPathItem =  listAllPathItem.get(listPosPath.get(i));
                List<Integer> list = customPathItem.getPathItem().getPath();
                double timePath = customPathItem.getTime();
                
                for (int j =0; j < list.size(); j++) {
                    int s = list.get(j);
                    //Tính lượng năng lượng cần giảm đi của sensor s
                    float energy = (float)(getEnergyConsumer(list, s) * timePath* (1- ratio));
                    updateEnergyOfSensor(listEnergy,s, energy);
                    
                }
                
                //Change time of Path contain Sensor
                customPathItem.setTime(timePath*ratio);

            }
            
            //Xoa phan tu có năng lượng sử dụng lớn nhất trong Listenergy
            listEnergy.remove(0);
            if (listEnergy.isEmpty()) return;
            
            //Sort lại list energy theo thu tu giam dan
            Collections.sort(listEnergy, new Comparator<EnergyItem>(){
            @Override
               public int compare(EnergyItem o1, EnergyItem o2) {
                   float size1 = o1.getEnergyUse();
                   float size2 = o2.getEnergyUse();
                   
                   return Float.compare(size2, size1);
               }
            
            });
            
            //Gan lai gia tri Max cua listEnergy
            MaxEnergyInList = listEnergy.get(0).getEnergyUse();
            
            
        }
        
    }
     
    void updateEnergyOfSensor(List<EnergyItem> listEnergy, int sensor, float energy) {
        for (int i =0 ; i < listEnergy.size(); i++) {
            if (listEnergy.get(i).getId() == sensor) {
                listEnergy.get(i).subEnergyUse(energy);
                break;
            }
        }
    }
     
    void FindListSensorInAllPath(List<List<PathItem>> returnListY, List<Integer> listSenSor) {
        boolean checkExit[];
        checkExit = new boolean[N];
        for (int i = 0; i < returnListY.size(); i++) {
            List<PathItem> listPath = returnListY.get(i);
            for (int j =0; j < listPath.size();j++) {
                PathItem path = listPath.get(j);
                List<Integer> list = path.getPath();
                for (int k =0; k < list.size();k++) {
                    checkExit[list.get(k)] = true;
                }
            } 
        }
        
        listSenSor.clear();
        for (int i =0; i < checkExit.length; i++) {
            if (checkExit[i]) {
                listSenSor.add(i);
            }
        }
    }
    
    boolean CheckExitListX(List<List<Integer>> listX, List<Integer> setX, int postionX) { //checked
        Collections.sort(setX);
        for (int i = 0 ;i<listX.size();i++) {
            List<Integer> tempX = listX.get(i);
            if (tempX.size() != setX.size()) {
                continue;
            } else {
                Collections.sort(tempX);
                int count = 0;
                for (int j =0;j<tempX.size();j++) {
                    if (!Objects.equals(setX.get(j), tempX.get(j))) {
                        break;
                    } else {
                        count++;
                    }
                }
                if (count == setX.size()) {
                    postionX = i;
                    return true;
                }
            }
            
             
        }
        return false;
    }
    
    int CheckExitListTargetInSaveList(List<Integer> listTarget, List<Integer> listSink) {
        int posTarget = -1;
        for(int i =0 ; i< SaveListTarget.size();i++) {
            List<Integer> list = SaveListTarget.get(i);
            if (list.size() == listTarget.size()) {
                int count =0;
                for (int j =0 ; j < listTarget.size();j++) {
                    if (list.get(j) != listTarget.get(j)) {
                        break;
                    } else {
                        count++;
                    }
                }
                
                if (list.size() == count) {
                    posTarget = i;
                }
            }
        }
        if (posTarget != -1 && posTarget < SaveListSink.size()) {
            List<Integer> listS = SaveListSink.get(posTarget);
            if (listS.size() == listSink.size()) {
                int count1 = 0;
                for (int j = 0; j < listSink.size(); j++) {
                    if (listS.get(j) != listSink.get(j)) {
                        break;
                    } else {
                        count1++;
                    }
                }

                if (listS.size() == count1) {
                    return posTarget;
                }
            }
        }
        
        return -1;
    }
    
    void CoppyListToSave(List<List<PathItem>> ListY, List<List<Double>> listTi, List<Integer> listTarget,List<Integer> listSink ) {
        List<List<PathItem>> newListY = new ArrayList<>();
        List<List<Double>> newlistTi = new ArrayList<>();
        List<Integer> newListTarget = new ArrayList<>();
        List<Integer> newListSink = new ArrayList<>();
        //Coppy ListY
        for (int i = 0 ;i < ListY.size();i++) {
            List<PathItem> pathY = ListY.get(i);
            List<PathItem> newPathY = new ArrayList<>();
            for (int j =0; j< pathY.size();j++) {
                newPathY.add(pathY.get(j));
            }
            newListY.add(pathY);
        }
        
        //Coppy ListTi
        for (int i =0 ; i < listTi.size(); i++) {
            List<Double> Ti = listTi.get(i);
            List<Double> newTi = new ArrayList<>();
            for (int j = 0; j < Ti.size();j++) {
                double t = Ti.get(j);
                newTi.add(t);
            }
            newlistTi.add(newTi);
        }
        
        //Coppy ListTarget
        for (int i =0; i < listTarget.size(); i++) {
            int target = listTarget.get(i);
            newListTarget.add(target);
        }
        
        //Coppy ListSink
        for (int i =0; i < listSink.size(); i++) {
            int sink = listSink.get(i);
            newListSink.add(sink);
        }
        
        SaveListofListY.add(newListY);
        SaveListofListTi.add(newlistTi);
        SaveListTarget.add(newListTarget);
    }
    
    void GetListFromSave(List<List<PathItem>> ListY, List<List<Double>> listTi, int pos) {
        if (pos >= SaveListofListY.size()) return;
        
        List<List<PathItem>> saveListY = SaveListofListY.get(pos);
        List<List<Double>> saveListTi = SaveListofListTi.get(pos);
        
        
        //Get pathY
        for (int i = 0 ; i < saveListY.size(); i++) {
            List<PathItem> savepathY = saveListY.get(i);
            List<PathItem> pathY = new ArrayList<>();
            
            for (int j = 0; j < savepathY.size(); j++) {
                pathY.add(savepathY.get(j));
            }
            ListY.add(pathY);
        }
        
        //Get listTi
        for (int i =0 ; i < saveListTi.size();i++) {
            List<Double> Ti = new ArrayList<>();  
            List<Double> saveTi = saveListTi.get(i);
            
            for (int j = 0; j< saveTi.size(); j++) {
                double t = saveTi.get(j);
                Ti.add(t);
            }
            listTi.add(Ti);
                
        }
        
    }
    
    
}
