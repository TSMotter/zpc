# Zephyr producer consumer with threads

```
west build --pristine
west flash
```

- Adapt uart structure to use simple ring buffer inserting byte by byte
- Integrate protobuf
- Maybe make it print back the decoded message