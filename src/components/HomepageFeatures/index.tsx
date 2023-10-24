import React from "react";
import clsx from "clsx";
import styles from "./styles.module.css";
import { Link } from "react-router-dom";
import { useHistory } from "react-router-dom";

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<"svg">>;
  description: JSX.Element;
  url: string;
};

const FeatureList: FeatureItem[] = [

    {
      title: 'CITROS Web',
      url: "vova",
      Svg: require('@site/static/img/citros_home_web.svg').default,
      description: (
        <>
          CITROS Web offers a full platform to create and run simulations, 
          investigate and analyse results, collaborate with your team with a simple UIץ
        </>
      ),
    },
    {
      title: 'CITROS CLI',
      url: "vova1",
      Svg: require('@site/static/img/citros_home_cli.svg').default,
      description: (
        <>
          CITROS CLI offers ROS 2 developers a seamless interface to launch 
          multiple ROS simulations for a specific project with just a single command.
        </>
      ),
    },
    {
      title: 'CITROS Data Analysis',
      url: "vova"2,
      Svg: require('@site/static/img/citros_home_analysis.svg').default,
      description: (
        <>
          CITROS Data Analysis offers you to data query, analyse and visualize it. 
          With its extensive features, you can quickly and easily extract valuable insights from your data.
        </>
      ),
    },
];

function Feature({ title, Svg, description, url }: FeatureItem) {
  const history = useHistory();
  return (
    <div
      className={clsx("col col--4")}
      onClick={() => {
        history.push(url);
      }}
      style={{ cursor: "pointer" }}
    >
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
