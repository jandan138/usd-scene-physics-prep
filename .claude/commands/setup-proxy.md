# Setup PJLab Proxy for GitHub Access

When GitHub HTTPS connections fail (TLS errors, timeout, etc.), set the PJLab proxy environment variables and retry.

## Steps

1. Set proxy environment variables for the current shell session:

```bash
export https_proxy=https://zhuzihou:VBIStWwOFfRgsbJzWlpjNbNxMKMEKRuFkcwIzFOxZbH15lohbROPxq55ShQC@aliyun-proxy.pjlab.org.cn:13128
export http_proxy=https://zhuzihou:VBIStWwOFfRgsbJzWlpjNbNxMKMEKRuFkcwIzFOxZbH15lohbROPxq55ShQC@aliyun-proxy.pjlab.org.cn:13128
export HTTP_PROXY=https://zhuzihou:VBIStWwOFfRgsbJzWlpjNbNxMKMEKRuFkcwIzFOxZbH15lohbROPxq55ShQC@aliyun-proxy.pjlab.org.cn:13128
export HTTPS_PROXY=https://zhuzihou:VBIStWwOFfRgsbJzWlpjNbNxMKMEKRuFkcwIzFOxZbH15lohbROPxq55ShQC@aliyun-proxy.pjlab.org.cn:13128
```

2. Ensure the git remote uses HTTPS (not SSH):

```bash
git remote set-url origin https://github.com/jandan138/usd-scene-physics-prep.git
```

3. Retry the failed git operation (push, pull, fetch, clone, etc.)

## Notes

- The proxy uses HTTPS protocol for both `http_proxy` and `https_proxy` (IT requirement)
- Both lowercase and uppercase variants must be set for compatibility
- This proxy is specific to the PJLab Aliyun environment
