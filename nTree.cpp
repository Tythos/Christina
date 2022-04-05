/*
 *  nTree.cpp
 *  structureTest
 *
 *  Created by Brian Kirkpatrick on 3/10/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#include <iostream>
#include "nTree.h"

using namespace std;

// --- Constructors --- //

template <class T> nTree<T>::nTree() {
	// A default nTree node will be the blank head of its tree
	pItem = NULL;
	parent = NULL;
	level = 0;
	nodeId = 0;
}

template <class T> nTree<T>::nTree(T * newItem) {
	// Creates a new nTree node with the given item
	parent = NULL;
	level = 0;
	nodeId = 0;
	pItem = newItem;
}

template <class T> nTree<T>::~nTree() {
	// Do not allow deletion of parent or linked item
	pItem = NULL;
	parent = NULL;
}

// --- Node-level functions --- //
template <class T> nTree<T> * nTree<T>::getNode(int requestedId) {
	// Returns a pointer to the requestes node in this node's subtree
	nTree<T> * toReturn = NULL;
	if (!isNode(requestedId)) {
		toReturn = NULL;
	} else {
		if (this->nodeId == requestedId)
			toReturn = this;
		else {
			int i = 0;
			int kidCount = children.length();
			while(i < kidCount && toReturn == NULL) {
				toReturn = children.get(i)->getNode(requestedId);
				i++;
			}
		}
	}
	return toReturn;
}

template <class T> nTree<T> * nTree<T>::headNode() {
	// Returns a pointer to the head node of this node's tree
	nTree<T> * toReturn = this;
	while(toReturn->parent != NULL) {
		toReturn = toReturn->parent;
	}
	return toReturn;
}

template <class T> int nTree<T>::numChildren() {
	// Returns the number of nodes below this node, at all generations
	int toReturn = children.length();
	int kidCount = toReturn;
	for (int i = 0; i < kidCount; i++) {
		toReturn += children.get(i)->numChildren();
	}
	return toReturn;
}

template <class T> int nTree<T>::getLevel() {
	// Returns the level of this node
	return this->level;
}

template <class T> void nTree<T>::printNode() {
	// Prints out a summary of this node and its children (indented)
	if (pItem != NULL) {
		for (int i = 0; i < level; i++) {
			cout << "\t";
		}
		cout << nodeId << ": " << (*pItem) << " (" << level << ")" << endl;
	}
	int kidCount = children.length();
	for (int j = 0; j < kidCount; j++) {
		children.get(j)->printNode();
	}
}

// --- Tree-level functions --- //
template <class T> T * nTree<T>::getItem(int nodeId) {
	// Returns the address of the item corresponding to the given node
	nTree * myNode = this->getNode(nodeId);
	return myNode->pItem;
}

template <class T> bool nTree<T>::isNode(int testId) {
	// Returns true if the given id corresponds to an existing node
	bool toReturn = false;
	if (nodeId == testId) toReturn = true;
	int kidCount = children.length();
	for (int i = 0; i < kidCount; i++) {
		if (children.get(i)->isNode(testId)) toReturn = true;
	}
	return toReturn;
}

template <class T> int nTree<T>::nextAvailableId() {
	// Returns the first unused id in the tree
	int testId = 0;
	int numNodes = this->getSize();
	while(testId < numNodes && this->isNode(testId)) {
		testId++;
	}
	return testId;
}

template <class T> int nTree<T>::getSize() {
	// Returns the total number of nodes in the tree
	if (pItem == NULL) return 0;
	return this->numChildren() + 1;
}

template <class T> int nTree<T>::addChild(T * newItem, int parentId) {
	// Adds a child node to the node with the given id
	int toReturn = 0;
	nTree<T> * newParent = this->getNode(parentId);
	if (newParent == NULL) {
		// Parent does not exist; add to head node
		toReturn = addChild(newItem);
	} else {
		toReturn = newParent->addChild(newItem);
	}
	return toReturn;
}

template <class T> int nTree<T>::addChild(T * newItem) {
	// Item is added as child of this node, or as this node item if blank
	int toReturn = 0;
	if (pItem == NULL) {
		pItem = newItem;
		toReturn = nodeId;
	} else {
		nTree<T> * treeHead = headNode();
		nTree<T> * newChild = new nTree<T>();
		newChild->pItem = newItem;
		newChild->parent = this;
		newChild->level = level + 1;
		toReturn = newChild->nodeId = treeHead->nextAvailableId();
		children.add(newChild);
	}
	return toReturn;
}

template <class T> void nTree<T>::removeNode(int requestedId) {
	// Deletes the node and children with the given id (but not the item)
	static int callCount = 0;
	callCount++;
	nTree<T> * targetNode = this->getNode(requestedId);
	nTree<T> * targetKid = NULL;
	nTree<T> * targetParent = targetNode->parent;
	if (targetNode != NULL) {
		// Remove children
		int kidCount = targetNode->children.length();
		for (int i = 0; i < kidCount; i++) {
			targetKid = targetNode->children.get(i);
			removeNode(targetKid->nodeId);
		}
		
		// Remove from parent list, if there is one
		if (targetNode->parent != NULL) {
			targetParent = targetNode->parent;
			kidCount = targetParent->children.length();
			for (int j = 0; j < kidCount; j++) {
				if (targetParent->children[j].nodeId == requestedId) {
					targetParent->children.remove(j);
				}
			}
		}
		
		// Delete (or reset, if top of tree)
		if (targetNode->level == 0) {
			pItem = NULL;
			parent = NULL;
			level = 0;
			nodeId = 0;
		} else {
			targetNode->parent = NULL;
			if (targetNode != NULL) delete targetNode;
		}
	}
}

template <class T> void nTree<T>::merge(nTree<T> * subTree, int parentId) {
	// Inserts head node under the given parent, and recursively inserts kids
	nTree<T> * targetNode = this->getNode(parentId);
	nTree<T> * targetKid = NULL;
	if (targetNode != NULL) {
		int newId = targetNode->addChild(subTree->pItem);
		int kidCount = subTree->children.length();
		for(int i = 0; i < kidCount; i++) {
			merge(subTree->children.get(i), newId);
		}
	}
}

template <class T> void nTree<T>::printNode(int nodeId) {
	// Prints the item stored in the given node
	nTree<T> * targetNode = this->getNode(nodeId);
	T * toPrint = targetNode->pItem;
	if (targetNode != NULL) {
		cout << *toPrint << endl;
	}
}

template <class T> void nTree<T>::printTree() {
	// Prints out entire contents of tree
	this->printNode();
}

template <class T> void nTree<T>::reset() {
	// Clears all contents of tree and resets ids, counters, pointers, etc
	removeNode(nodeId);
}
