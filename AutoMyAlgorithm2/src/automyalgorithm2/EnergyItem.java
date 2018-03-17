/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package automyalgorithm2;

import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author Hieu
 */
public class EnergyItem {
    
    int id ; //Id of Sensor
    float energyUse;
    List<Integer> listPosTarget; //List target co duong path chua id sensor
    List<Integer> listPosPath;//Position cua path tuong ung voi taget
    

    public EnergyItem() {
    }

    public EnergyItem(int id, float energyUse) {
        this.id = id;
        this.energyUse = energyUse;
        listPosTarget = new ArrayList<>();
        listPosPath = new ArrayList<>();
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public float getEnergyUse() {
        return energyUse;
    }

    public void setEnergyUse(float energyUse) {
        this.energyUse = energyUse;
    }
    
    public void addEnergyUse(float energy) {
        this.energyUse += energy;
    }
    
    public void subEnergyUse(float energy) {
        this.energyUse -= energy;
    }
    
    public void addPostion(int target,int postion) {
        listPosTarget.add(target);
        listPosPath.add(postion);
    }
    
    public List<Integer> getPosTargetList() {
        return listPosTarget;
    }
    
    public List<Integer> getPosPathList() {
        return listPosPath;
    }
}
