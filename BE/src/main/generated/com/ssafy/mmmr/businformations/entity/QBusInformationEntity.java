package com.ssafy.mmmr.businformations.entity;

import static com.querydsl.core.types.PathMetadataFactory.*;

import com.querydsl.core.types.dsl.*;

import com.querydsl.core.types.PathMetadata;
import javax.annotation.processing.Generated;
import com.querydsl.core.types.Path;


/**
 * QBusInformationEntity is a Querydsl query type for BusInformationEntity
 */
@Generated("com.querydsl.codegen.DefaultEntitySerializer")
public class QBusInformationEntity extends EntityPathBase<BusInformationEntity> {

    private static final long serialVersionUID = -935732559L;

    public static final QBusInformationEntity busInformationEntity = new QBusInformationEntity("busInformationEntity");

    public final NumberPath<Integer> id = createNumber("id", Integer.class);

    public final StringPath route = createString("route");

    public final NumberPath<Integer> routeId = createNumber("routeId", Integer.class);

    public final NumberPath<Integer> sequence = createNumber("sequence", Integer.class);

    public final StringPath station = createString("station");

    public final NumberPath<Integer> stationId = createNumber("stationId", Integer.class);

    public QBusInformationEntity(String variable) {
        super(BusInformationEntity.class, forVariable(variable));
    }

    public QBusInformationEntity(Path<? extends BusInformationEntity> path) {
        super(path.getType(), path.getMetadata());
    }

    public QBusInformationEntity(PathMetadata metadata) {
        super(BusInformationEntity.class, metadata);
    }

}

