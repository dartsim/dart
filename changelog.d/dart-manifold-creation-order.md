Section: Collision

  * **Determinism change:** The DART collision detector now orients each
    contact manifold by the order in which it created the two objects. It no
    longer uses their memory addresses, so contacts and warm starts no longer
    depend on heap layout. Results can change in the last bits where the two
    orders differed. `DARTCollisionObject` gains a member, so code that
    subclasses it must be rebuilt.
