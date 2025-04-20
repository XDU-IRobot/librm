import React from "react";
import Layout from "@theme/Layout";
import styles from "./styles.module.css";

export default function DoxygenPage() {
  return (
    <Layout>
      <div className={styles.doxygen_container}>
        <iframe src="/doxygen/index.html" className={styles.doxygen_main} />
      </div>
    </Layout>
  );
}
