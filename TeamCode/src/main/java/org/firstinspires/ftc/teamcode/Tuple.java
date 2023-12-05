package org.firstinspires.ftc.teamcode;

public class Tuple<K, V> {
    public final K key;
    public final V value;
    public Tuple(K key, V value) {
        this.key = key;
        this.value = value;
    }
    public K getKey() {
        return this.key;
    }
    public V getValue() {
        return this.value;
    }
}
