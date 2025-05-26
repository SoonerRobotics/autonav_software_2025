"use client";

import { Fragment } from "react";

type ClientSideProps = {
    children: React.ReactNode;
};

export default function ClientSide(props: ClientSideProps) {
    return (
        <Fragment>
            {props.children}
        </Fragment>
    );
}