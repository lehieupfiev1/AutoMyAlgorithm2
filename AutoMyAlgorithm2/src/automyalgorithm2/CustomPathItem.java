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
public class CustomPathItem {
    List<Integer> listId;
    PathItem pathItem;
    double time;

    public CustomPathItem() {
    }

    public CustomPathItem(PathItem pathItem) {
        this.pathItem = pathItem;
        listId = new ArrayList<>();
        time =0;
    }

    public CustomPathItem(List<Integer> listId, PathItem pathItem) {
        this.listId = listId;
        this.pathItem = pathItem;
        time =0;
    }

    public List<Integer> getListId() {
        return listId;
    }

    public void setListId(List<Integer> listId) {
        this.listId = listId;
    }

    public PathItem getPathItem() {
        return pathItem;
    }

    public void setPathItem(PathItem pathItem) {
        this.pathItem = pathItem;
    }

    public double getTime() {
        return time;
    }

    public void setTime(double time) {
        this.time = time;
    }
    
}
